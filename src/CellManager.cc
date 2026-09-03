#include "CellManager.h"

#include <iostream>
#include <fstream>
#include <numeric>
#include <iomanip>

namespace ORB_SLAM3 
{

//for execution time prediction
static double g_pending_pred_ms = -1.0;

CellManager& CellManager::getInstance()
{
    static CellManager instance;
    return instance;
}

void CellManager::incrementCell()
{
    elapsed_cells++;  // Increment the frame's elapsed Cells
}

bool CellManager::skipCell(const feature_extraction_state_t& cell)
{
    bool skip = false;

    // Register THIS level's grid dimensions. Lock-free: fixed-capacity atomic arrays,
    // so there is nothing to grow and nothing to serialise on the per-cell hot path.
    // Explicit on BOTH ends. `cell.level` is int and kMaxPyramidLevels is size_t, so
    // `cell.level < kMaxPyramidLevels` promotes the int to size_t and a negative
    // level becomes SIZE_MAX -- correctly rejected, but only by accident of
    // conversion, and the kind of implicit signed/unsigned comparison that changes
    // meaning the moment a type does.
    if (cell.level >= 0 && static_cast<size_t>(cell.level) < kMaxPyramidLevels)
    {
        level_rows[cell.level].store(cell.nRows, std::memory_order_relaxed);
        level_cols[cell.level].store(cell.nCols, std::memory_order_relaxed);
        int prev = max_level_seen.load(std::memory_order_relaxed);
        while (static_cast<int>(cell.level) > prev &&
               !max_level_seen.compare_exchange_weak(prev, static_cast<int>(cell.level),
                                                     std::memory_order_relaxed))
        { /* prev is reloaded by compare_exchange_weak */ }
    }

    // Why the assignment above matters: without it the vector is only ever resized,
    // so every pyramid_level_t stays default-constructed at 0x0 and the mask search
    // in endFrame() degenerates -- largest_mask = max(0,0)+1 = 1, and the loop
    // 'for(mask = 2; mask < largest_mask; ...)' never executes, so FOV_MASK is left
    // at whatever endFrame last stored (2) and the extractor is starved to ~102 of
    // ~1352 cells on every frame, on any hardware. The authors' own committed
    // cellManager.txt shows real dimensions (Level 0: 20x12, ...).

    // cells_per_level needs no lazy initialisation: it is a zero-initialised array.

    // NOTE: Do NOT increment per-level counters here - we haven't decided
    // whether the cell will be skipped yet. The FOV mask and skip_frames
    // checks below determine whether this cell is processed. We will
    // increment the per-level counter after the skip decision so the
    // mask can actually prevent cells from being counted.

    // Static masking patterns. Evaluated BEFORE the adaptive FOV mask and independently
    // of `enableOasis`: these are the comparison arms, so they must work with the
    // adaptive controller off. A pattern is a pure function of (level, row, col) --
    // no shared state, no RNG object -- so the two stereo extractor threads compute
    // identical decisions without coordination.
    //
    // NOT "a run is bit-reproducible", which this comment used to claim. The MASK is
    // reproducible -- the same cell yields the same decision every time, which is the
    // property that matters here -- but a RUN is not: the extractor threads race
    // elsewhere, and two runs of the same binary on the same input produce different
    // trajectory bytes (measured: up to 3.9% apart; `cmp` of two MH01 runs exits 1).
    // Conflating the two is what produced three rounds of false off-equivalence claims.
    const int pattern = maskPattern.load(std::memory_order_relaxed);
    if( pattern != MASK_PATTERN_OFF )
    {
        // THE FIELDS ARE TRANSPOSED UPSTREAM. ORBextractor::ComputeKeyPointsOctTree
        // builds the cell as `{.col=i, .row=j}` where `i` is the ROW loop index (it
        // drives iniY, the vertical coordinate) and `j` is the COLUMN loop index (it
        // drives iniX). So `cell.col` holds a row number and `cell.row` holds a column
        // number -- the names mean the opposite of what they say. Present since
        // 8862ba1, i.e. this is the artifact's own wiring, not something introduced
        // here.
        //
        // Reading through the transposition HERE rather than fixing it at the source
        // is deliberate: the FOV block below indexes the same two fields, and the
        // paper's own published numbers were produced with that geometry. Correcting
        // ORBextractor would silently move the FOV window and invalidate every Phase 1
        // result already collected. Renaming locally is behaviour-preserving for
        // everything except the two stripe patterns, which is precisely what needs to
        // change.
        //
        // Left uncorrected, EuRoC_mask_vstripes.yaml produced horizontal stripes and
        // EuRoC_mask_hstripes.yaml produced vertical ones, so both columns would have
        // been published against the wrong row of tab:rpi-pose-error.
        const int true_row = cell.col;   // i: vertical index,   0 .. nRows-1
        const int true_col = cell.row;   // j: horizontal index, 0 .. nCols-1

        switch( pattern )
        {
            case MASK_PATTERN_CHECKER:
                // Symmetric under the transposition, so this one was always correct.
                skip = ((true_row + true_col) % 2) != 0;
                break;
            case MASK_PATTERN_VSTRIPES:
                // Vertical stripes = alternating COLUMNS masked.
                skip = (true_col % 2) != 0;
                break;
            case MASK_PATTERN_HSTRIPES:
                // Horizontal stripes = alternating ROWS masked.
                skip = (true_row % 2) != 0;
                break;
            case MASK_PATTERN_RANDOM:
            {
                // Deterministic hash, NOT a random number generator. std::rand or a
                // shared engine would give the two extractor threads different
                // sequences and make consecutive runs differ -- the pattern has to be
                // a stable property of the cell, exactly as the other three are.
                // splitmix64 finalizer over the packed key.
                uint64_t k = (static_cast<uint64_t>(maskRandomSeed.load(std::memory_order_relaxed)) << 40)
                           ^ (static_cast<uint64_t>(cell.level) << 32)
                           ^ (static_cast<uint64_t>(static_cast<uint32_t>(true_row)) << 16)
                           ^  static_cast<uint64_t>(static_cast<uint32_t>(true_col));
                k ^= k >> 30; k *= 0xbf58476d1ce4e5b9ULL;
                k ^= k >> 27; k *= 0x94d049bb133111ebULL;
                k ^= k >> 31;
                const int pct = maskDensityPct.load(std::memory_order_relaxed);
                skip = static_cast<int>(k % 100ULL) >= pct;   // keep `pct`% of cells
                break;
            }
            default:
                break;
        }
    }

    // The adaptive FOV mask applies only when the OASIS controller was REQUESTED and
    // has warmed up. `enableOasis` alone means only "warmed up" -- it is set
    // unconditionally at the end of the first endFrame -- so gating on it alone would
    // silently layer the adaptive mask on top of a static pattern from frame 2
    // onwards and corrupt the comparison the patterns exist to make.
    if( enableOasis && oasisRequested.load(std::memory_order_relaxed) )
    {
        const int maskWidth = FOV_MASK.width;
        const int maskHeight = FOV_MASK.height;
        // using the center of the cell to determine if it's in the FOV_MASK
        const int center_row = cell.nRows/2;
        const int center_col = cell.nCols/2;

        if( cell.row < center_row - maskHeight/2 || cell.row > center_row + maskHeight/2 )
        {
            skip = true;
        }

        if( cell.col < center_col - maskWidth/2 || cell.col > center_col + maskWidth/2 )
        {
            skip = true;
        }
    
        // check if we're skipping this cell, based on the number of frames we need to skip
        if( skip_frames )
        {
            skip = true;
        }
    }

    if( skip )
    {
        // We're skipping this cell, so we need to decrement the frame's elapsed Cells
        // to keep out this from the tracking stats
        elapsed_cells--;
    }

    // Only count this cell for the pyramid level if it is NOT skipped.
    if (!skip)
    {
        if (cell.level >= 0 && static_cast<size_t>(cell.level) < kMaxPyramidLevels)
            cells_per_level[cell.level].fetch_add(1, std::memory_order_relaxed);
    }

    return skip;
}

// Signal the end of a frame and reset elapsed Cells
void CellManager::endFrame(const double& frame_num, double actualFrameTime,
                           FrameAttemptOutcome outcome, uint64_t attemptId)
{

    const bool schema2 = dropAccountingFix.load(std::memory_order_relaxed);
    const bool pre_tracking_drop = schema2 && outcome != FrameAttemptOutcome::ReachedTracking;

    // if we're skipping frames, note it and decrement the number of frames we need to skip
    //
    // GATED ON oasisRequested. skip_frames is the ADAPTIVE actuator's state: the budget
    // code below sets it from how far over budget a frame ran, and skipCell consumes it
    // only under (enableOasis && oasisRequested). endFrame, however, runs on EVERY frame
    // of EVERY run -- so on a static-mask run, where nothing ever acts on skip_frames,
    // this still decremented it, printed "Skipped/Dropped frame", and wrote skipped=1
    // into exec_time_eval.txt for frames that were fully processed.
    //
    // That column is the dropped-frame metric the paper reports, so a static-mask run
    // would have published fabricated drops. Observed transition before the fix:
    //   frame=1 actual=151ms skipped=0 -> next_skip_frames=2
    //   frame=2 actual=40ms  skipped=1 (but the frame WAS processed)
    //   frame=3 actual=40ms  skipped=1 (likewise)
    const bool oasis_active = enableOasis && oasisRequested.load(std::memory_order_relaxed);
    bool was_skipped = false;
    if( !pre_tracking_drop && oasis_active && skip_frames )
    {
        std::cout << "Skipped/Dropped frame " << frame_num << std::endl;
        was_skipped = true;
        skip_frames--;
    }
    else if( !pre_tracking_drop && !oasis_active )
    {
        // Keep the actuator inert rather than merely unread, so no later frame inherits
        // a nonzero count and the budget path below cannot branch on stale state.
        skip_frames = 0;
    }

    //execution time prediction-------------------------
    // Schema 2 truncates on its own first open, then appends. Schema 1 deliberately
    // preserves the artifact's legacy append behavior.
    static bool schema2_initialized = false;
    static std::atomic<bool> legacy_header_written(false);
    const bool first_schema2_open = schema2 && !schema2_initialized;
    std::ofstream et_log("exec_time_eval.txt",
                         first_schema2_open ? std::ios::trunc : std::ios::app);

    if (!et_log.is_open()) {
         std::cerr << "Error opening exec_time_eval.txt!" << std::endl;
        return;
    }

    const bool write_header = schema2 ? first_schema2_open
                                      : !legacy_header_written.exchange(true);
    if (write_header) {
        et_log << "frame,predicted_ms,actual_ms,avg_cells_per_frame,actual_cells,skipped,mask_w,mask_h";
        for (int l = 0; l < 8; ++l) et_log << ",L" << l;
        et_log << "\n";
        if(schema2)
            schema2_initialized = true;
    } 

    if(schema2 || g_pending_pred_ms >= 0.0)
    {        
       et_log << std::fixed << std::setprecision(6)
           << frame_num << "," << (pre_tracking_drop ? -1.0 : g_pending_pred_ms) << "," << actualFrameTime << ","
           << getAverageCellsPerFrame() << "," << (pre_tracking_drop ? 0 : elapsed_cells.load(std::memory_order_relaxed)) << ","
           << (pre_tracking_drop ? 2 : (was_skipped ? 1 : 0)) << ","
           << (pre_tracking_drop ? 0 : FOV_MASK.width) << ","
           << (pre_tracking_drop ? 0 : FOV_MASK.height);

        // Append per-level cell counts (L0..L7). If fewer levels exist, pad with zeros.
        for (int l = 0; l < 8; ++l)
            et_log << "," << (pre_tracking_drop ? 0 :
                              (static_cast<size_t>(l) < kMaxPyramidLevels
                               ? cells_per_level[l].load(std::memory_order_relaxed) : 0));

        et_log << "\n";
        et_log.flush();



    if(!pre_tracking_drop)
        g_pending_pred_ms = -1.0;
    }

    //end exection time prediction log--------------------------


    // A schema-2 pre-tracking drop is only an observation. It must not consume a
    // pending prediction, adaptive skip credit, cell history, or controller state.
    if(pre_tracking_drop)
    {
        // The timestamp is printed EXACTLY as the CSV's frame column prints it (fixed,
        // 6 decimals), because the runner pairs this event with that row by text. The
        // stream's own precision is whatever the example left it at -- 17 significant
        // digits in stereo_inertial_euroc, the default 6 elsewhere, which would print
        // 1.40364e+09 and pair with nothing.
        std::cout << "R4_EXEC attempt=" << attemptId << " timestamp="
                  << std::fixed << std::setprecision(6) << frame_num
                  << " skipped=2 overhead_ms=" << actualFrameTime << std::endl;
        std::cout.unsetf(std::ios_base::floatfield);
        return;
    }

    // if no Cells were recorded, return
    if(elapsed_cells == 0)
    {
        // NOTE, as-shipped and deliberately left alone: this formula is
        // `1/ms x cells`, not `ms / (ms/cell)` -- dimensionally it is not a cell count at
        // all, and it disagrees with the budget computed 30 lines below. It is reached only
        // on a frame that extracted zero cells, and it is STILL UNOBSERVABLE even now that
        // frame_budget is a real member: this branch returns before printStats, and the
        // next frame that reaches the publish site below overwrites the value before its
        // own printStats runs. (An earlier version of this comment claimed the value had
        // become visible; the 2026-09-02 read-only investigation of the D40 runs showed it
        // never reaches cellManager.txt.) Flagged rather than corrected: changing it would
        // change as-shipped behaviour on a path this experiment does not exercise.
        frame_budget = static_cast<int>(1.0 / actualFrameTime * getAverageCellsPerFrame());
        return;
    }

    if( skip_frames )
    {
        // we're skipping frames, so we don't need to calculate the budget
        // just reset the elapsed cells and return
        elapsed_cells = 0;
        return;
    }

    cells_per_frame.push_back(elapsed_cells);

    // IS THIS A STEREO FRAME? Two answers, and the shipped one is a feedback loop.
    //
    // As shipped, stereo is INFERRED from the post-mask cell count: if this frame
    // processed more than 1.8x a full level-0 grid, both images must have been done.
    // That holds only while the mask is open. Once the controller masks below the
    // threshold -- 432 cells on EuRoC, where level 0 is 20x12 -- a genuinely stereo frame
    // is classified MONOCULAR, the `budget_cells /= 2` below is skipped, and the budget
    // DOUBLES. The mask reopens to full grid, the cell count climbs back over the
    // threshold, the halving returns, and the mask collapses again.
    //
    // It is a positive feedback loop and it closes. Measured, EuRoC MH01 at D40:
    //   after a mono-classified frame  (cells<=432, n= 518): next mask full grid  64%
    //   after a stereo-classified frame (cells >432, n=1550): next mask full grid  39%
    //   mask_w histogram bimodal at 1-15 and 22, NOTHING between 16 and 21
    //   mean frame-to-frame |delta mask_w| = 8.1 cells; 43% of transitions jump >=10
    // Consecutive frames read (mask 1, 12 cells) then (mask 22, 1352 cells) at 133.5 ms.
    // It also silently bypasses the overBudgetFix correction below, which sits inside the
    // same guard -- on 31% of over-budget frames, i.e. exactly where that arm is measured.
    //
    // Under System.oasisStereoFix the sensor type is used instead. Settings knows it
    // (Settings.cc: sensor_ == System::STEREO || IMU_STEREO) and it is a constant of the
    // run, so no amount of masking can change the answer.
    bool stereo_slam = false;
    if( stereoFix.load(std::memory_order_relaxed) )
    {
        stereo_slam = stereoSensor.load(std::memory_order_relaxed);
    }
    else
    {
        const int top_level = max_level_seen.load(std::memory_order_relaxed);
        if (top_level >= 0) {
            stereo_slam = (elapsed_cells > (level_rows[0].load(std::memory_order_relaxed)
                                          * level_cols[0].load(std::memory_order_relaxed)) * 1.8);
        }
    }

    // Using actual time elapsed to do frame as the budget for the next frame
    const double time_per_cell = ( actualFrameTime / getAverageCellsPerFrame());

    // We know that the frame should take 50ms, so we can calculate the budget
    // based on when we're done with the frame, assumming actual time elapsed is < 50ms
    // if it's > 50ms, we'll have to adjust the budget accordingly
    const double frame_time = 50.0f; //ms
    // budget_cells, NOT frame_budget. A local named frame_budget SHADOWED the member of
    // the same name (CellManager.h:53) for the whole remainder of endFrame, so printStats
    // read the member -- which the normal path never writes. "Frame Budget in Cells: 0" is
    // consequently the ONLY value in every cellManager.txt this project has ever produced,
    // across 1371 runs. The budget is the central observable of the deadline experiment,
    // and it was never once recorded.
    double budget_cells = static_cast<int>( frame_time / time_per_cell );
    
    // if we're doing stereo slam, we need to halve the budget (processing two images per frame)
    if( stereo_slam )
        budget_cells /= 2;

    if( actualFrameTime > frame_time )   // if we're over budget, adjust the frame budget for the next frame
    {
        double frame_time_remaining = actualFrameTime;
        size_t frames_over_budget = 0;
        
        // see how many frames we're over budget
        while( frame_time_remaining > frame_time )
        {
            frames_over_budget++;
            frame_time_remaining -= frame_time;
        }

        // adjust number of frames over budget to account for 'dropped frames'
        skip_frames = static_cast<int>(frames_over_budget);

        // Assuming we're resuming at the same rate, we can calculate the remaining budget
        // We'll consume one of the skipped frames by accounting for the extra time needed
        // in this frame!
        const double remaining_budget = (2*frame_time) - ( actualFrameTime - frame_time * (skip_frames -1) );
        if( skip_frames ) skip_frames--; // decrement!
        budget_cells =  static_cast<int>( remaining_budget / time_per_cell);

        // THE STEREO HALVING IS MISSING FROM THIS BRANCH AS SHIPPED.
        //
        // Twenty lines above, the under-budget path halves the budget for stereo because
        // the frame processes two images while the mask-selection loop below counts cells
        // for ONE. This branch overwrites the budget and does not re-apply that halving,
        // so crossing the deadline hands the selector a budget that is twice what the same
        // frame time would have produced had it stayed under. A review seat found it and
        // reproduced the arithmetic against the one 63.92 ms frame in the TUM-VI
        // collection: the next predicted_ms became 36.07, exactly as the unhalved formula
        // gives.
        //
        // Gated rather than simply corrected, because this project reproduces a published
        // artifact: `System.oasisOverBudgetFix: 1` opts in, and with the key absent the
        // behaviour is bit-for-bit what the paper's code does. That lets one binary serve
        // both arms of the comparison instead of forcing a third build.
        if( stereo_slam && overBudgetFix.load(std::memory_order_relaxed) )
            budget_cells /= 2;
    }

    //store estimated time prediction for next frame
    g_pending_pred_ms = budget_cells * time_per_cell;

    // PUBLISH THE BUDGET AND THE BRANCH so printStats and the analysis can see them. Until
    // now neither was observable: the budget went into a shadowing local, and nothing
    // recorded whether the over-budget path had run.
    frame_budget = static_cast<int>( budget_cells );
    last_over_budget = ( actualFrameTime > frame_time );

    // Iterate through each mask size, calculating the number
    // of cells the proposed mask will cover, and compare that against
    // the frame budget, to determine if the mask should become FOV_MASK
    // starting with a 2x2 mask, and increasing in size, until budget is exceeded
    // Snapshot the atomic arrays into a plain local, so the mask search below sees a
    // stable view even if extraction for the next frame has already begun.
    std::vector<pyramid_level_t> levels;
    {
        const int top = max_level_seen.load(std::memory_order_relaxed);
        for (int i = 0; i <= top && static_cast<size_t>(i) < kMaxPyramidLevels; ++i)
            levels.push_back(pyramid_level_t{ level_rows[i].load(std::memory_order_relaxed),
                                              level_cols[i].load(std::memory_order_relaxed) });
    }
    if(levels.size() < 1)
    {
        // we don't have any pyramid levels, so we can't set the FOV_MASK
        std::cout << "No pyramid levels found, can't set FOV_MASK" << std::endl;
        return;
    }
    const int largest_mask = std::max(levels[0].nRows, levels[0].nCols) + 1;
    FOV_MASK.height = largest_mask + 1;
    FOV_MASK.width = largest_mask + 1;

    for( int mask = 2; mask < largest_mask; mask++ )
    {
        const int maskWidth = mask;
        const int maskHeight = mask;

        int cells_in_mask = 0;
        // Go through each pyramid level and calculate the number of cells
        // that would be covered by the mask
        for( int level = 0; level < levels.size(); level++ )
        {
            const int cells_at_level = levels[level].nRows * levels[level].nCols;
            if( cells_at_level < (maskWidth * maskHeight) )
            {
                cells_in_mask += cells_at_level;
            }
            else
            {
                cells_in_mask += maskWidth * maskHeight;
            }
        }

        if( cells_in_mask < budget_cells )
        {
            // we're still in budget! keep going
            continue;
        }
        else
        {
            // We've found our mask size!
            // use the previous mask size to set the FOV_MASK
            const int prev_mask = mask - 1;
            FOV_MASK.height = prev_mask; 
            FOV_MASK.width = prev_mask;
            break;
        }
    }

    // We're warmed up and can start filtering cells!
    enableOasis = true;
    printStats(frame_num, actualFrameTime);

    // Reset for the next frame
    elapsed_cells = 0;
    
    // Reset the per-level counters for next frame
    for (auto& level_count : cells_per_level)
        level_count.store(0, std::memory_order_relaxed);
}

// Calculate average Cells per frame
double CellManager::getAverageCellsPerFrame() const 
{
    if (cells_per_frame.empty()) return 0.0;
    int total_cells = std::accumulate(cells_per_frame.begin(), cells_per_frame.end(), 0);
    return static_cast<double>(total_cells) / cells_per_frame.size();
}

// Debug print
void CellManager::printStats(const double& frame_num, const double& frameTimestamp) const
{
    // Open file in append mode
    std::ofstream file("cellManager.txt", std::ios::app);
    if (!file) {
        // Handle file open error
        std::cerr << "Failed to open cellManager.txt" << std::endl;
        return;
    }

    // Print out the pyramid levels (only once)
    static bool once = true;
    if (once)
    {
        file << " - maskPattern=" << maskPattern.load(std::memory_order_relaxed)
             << " densityPct=" << maskDensityPct.load(std::memory_order_relaxed)
             << " oasisRequested=" << (oasisRequested.load(std::memory_order_relaxed) ? 1 : 0)
             << "\n";
        file << " - Pyramid Level Cells: \n";
        {
            const int top = max_level_seen.load(std::memory_order_relaxed);
            for (int i = 0; i <= top && static_cast<size_t>(i) < kMaxPyramidLevels; i++)
            {
                file << "   - Level " << i << ": "
                     << level_cols[i].load(std::memory_order_relaxed) << "x"
                     << level_rows[i].load(std::memory_order_relaxed) << "\n";
            }
        }

        // Print out if oasis enabled.
        // BOTH flags, because `enableOasis` alone is the internal warmed-up latch that
        // endFrame sets true after frame 1 on EVERY run, masked or not -- so this line
        // printed "Oasis Enabled" on all four static-mask runs, directly above its own
        // `oasisRequested=0`. The adaptive FOV mask is gated on the conjunction
        // (see skipCell), so the label must be too, or a human sanity-checking a
        // pattern run is told OASIS was active when it was not.
        file << ((enableOasis && oasisRequested.load(std::memory_order_relaxed))
                 ? " - Oasis Enabled" : " - Oasis Disabled") << std::endl;
        once = false;
    }

    file << "Frame " << std::fixed << std::setprecision(9) << frame_num << " finished in " << frameTimestamp << " ms stats:\n";
    file << " - Recorded Frames: " << cells_per_frame.size() << "\n";
    file << " - Elapsed Cells: " << elapsed_cells << "\n";
    file << " - Average Cells Per Frame: " << getAverageCellsPerFrame() << "\n";
    file << " - Frame Budget in Cells: " << frame_budget << "\n";
    file << " - Over Budget: " << (last_over_budget.load(std::memory_order_relaxed) ? 1 : 0) << "\n";
    
    // Print out the FOV_MASK
    file << " - FOV Mask: " << FOV_MASK.width << "x" << FOV_MASK.height << "\n";

    // Print cells processed per pyramid level
    file << " - Cells per pyramid level:\n";
    {
        const int top = max_level_seen.load(std::memory_order_relaxed);
        for (int i = 0; i <= top && static_cast<size_t>(i) < kMaxPyramidLevels; i++)
            file << "   Level " << i << ": "
                 << cells_per_level[i].load(std::memory_order_relaxed) << " cells\n";
    }
}
}
