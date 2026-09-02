#pragma once

#include <array>
#include <atomic>
#include <vector>
#include <memory>
#include <chrono>

namespace ORB_SLAM3 
{

// TODO, this is a placeholder for now
// Plan is to adjust SLAM settings later
struct feature_extraction_settings_t
{
    // int mnScaleLevels;
    // float mfScaleFactor;
    // float mfLogScaleFactor;
    // std::vector<float> mvScaleFactors;
    // std::vector<float> mvInvScaleFactors;
    // std::vector<float> mvLevelSigma2;
    // std::vector<float> mvInvLevelSigma2;
};

struct feature_extraction_state_t
{
    // These are the variables we need in order to figure out
    // how much time we have to provision to featurizing the frame
    int level;
    int col;
    int row;

    int nLevels;
    int nCols;
    int nRows;
};

class CellManager 
{
private:
    std::atomic<int> elapsed_cells;
    std::vector<int> cells_per_frame;
    // Fixed-capacity atomic arrays, NOT vectors. skipCell() runs on the per-cell hot
    // path from both stereo extractor threads (~2700 calls/frame). A vector must be
    // resized to grow, concurrent resize() is undefined behaviour, and serialising
    // that with a mutex measurably perturbs the very quantity this instrumentation
    // exists to measure -- ~0.74 ms/frame, 3-5% of t_track, added to one arm only.
    // A fixed array indexed by level needs neither growth nor locking.
    // ORB-SLAM3 uses 8 pyramid levels; the capacity carries headroom and levels
    // beyond it are ignored rather than allowed to write out of bounds.
    static constexpr size_t kMaxPyramidLevels = 16;
    std::array<std::atomic<int>, kMaxPyramidLevels> cells_per_level{};  // cells processed per level
    // frame_budget is PUBLISHED for observability. Until 2026-09-02 a local of the same
    // name in endFrame shadowed it, so printStats read a member nothing wrote and every
    // cellManager.txt in this project recorded "Frame Budget in Cells: 0".
    std::atomic<int> frame_budget{0};
    // Which branch set it, so an actuation trace can distinguish the under-budget path
    // from the over-budget one instead of inferring it from the frame time.
    std::atomic<bool> last_over_budget{false};
    // Opt-in correction for the missing stereo halving in the over-budget branch. Default
    // FALSE so an unmodified config reproduces the published behaviour exactly; the
    // deadline experiment collects both arms from one binary by flipping this key.
    std::atomic<bool> overBudgetFix{false};
    std::atomic<bool> enableOasis = false;
    std::atomic<int> skip_frames = 0;


    struct pyramid_level_t
    {
        int nRows;    // number of rows in the grid at this level
        int nCols;    // number of columns in the grid at this level
    };

    struct mask_t
    {
        int width;
        int height;
    };

    // Static masking patterns, for the OASIS-vs-static comparison the paper describes
    // (tex/03-evaluation.tex:271) but does not ship code for. The paper's fig:masks
    // caption -- "Cells highlighted in green indicate cells to be processed for
    // feature extraction" -- fixes the quantum as the extractor GRID CELL, the same
    // one the adaptive FOV mask uses, applied per pyramid level so the active
    // fraction is scale-invariant.
    enum mask_pattern_t
    {
        MASK_PATTERN_OFF        = 0,
        MASK_PATTERN_CHECKER    = 1,
        MASK_PATTERN_VSTRIPES   = 2,
        MASK_PATTERN_HSTRIPES   = 3,
        MASK_PATTERN_RANDOM     = 4,
    };
    std::atomic<int>    maskPattern{ MASK_PATTERN_OFF };
    std::atomic<int>    maskRandomSeed{ 0 };
    // Active fraction for MASK_PATTERN_RANDOM. Default 0.5 because Random_Cell's
    // dropped-frame and FPS behaviour in the paper's own static_masks.csv is
    // statistically indistinguishable from checkerboard/vstripes/hstripes, which are
    // 50% active by construction. That is a WEAK inference, not a recovered value --
    // the shipped columns cannot discriminate density (spec section 4).
    std::atomic<int>    maskDensityPct{ 50 };

    // an instance of the MASK struct we'll use!
    mask_t FOV_MASK;

    // Grid dimensions per pyramid level, registered by skipCell(). Same rationale as
    // cells_per_level: fixed capacity, atomic, no locking on the hot path. Both
    // extractor threads write identical values for a given level, so relaxed stores
    // are sufficient -- there is no ordering dependency between them.
    std::array<std::atomic<int>, kMaxPyramidLevels> level_rows{};
    std::array<std::atomic<int>, kMaxPyramidLevels> level_cols{};
    // Highest level index seen, so readers know how much of the arrays is live.
    std::atomic<int> max_level_seen{-1};

public:

    // we'll use a singleton pattern so we can use this in multiple places 
    // (e.g. Frame, Tracking, etc.) while maintaining a single instance
    static CellManager& getInstance();

    // Configure the static masking pattern. Set from the settings file before
    // tracking starts.
    //
    // Pattern 0 leaves the masking DECISIONS unchanged from a build without this
    // feature -- NOT byte-identical behaviour, which this comment previously claimed and
    // which is not true in two ways. First, the extractor gate became
    // `enableOasis || CellManager::getInstance().maskPatternActive()`, so a pattern-off
    // run performs one singleton lookup and one relaxed atomic load per candidate cell
    // that it did not before. Second, byte-identical trajectories are unachievable for
    // ANY two runs here: the extractor threads race and two runs of the same binary
    // differ by up to 3.9%.
    //
    // What the off-equivalence check actually verifies is that skipCell is never CALLED
    // when pattern and OASIS are both off -- demonstrated by the absence of
    // cellManager.txt, since skipCell is the only thing that registers the pyramid
    // dimensions that file requires.
    // Whether the ADAPTIVE OASIS controller was requested in the settings file, as
    // distinct from `enableOasis` below, which means "the controller has warmed up".
    // Conflating the two is what made the static patterns dead code: the extractor
    // gated its skipCell call on the settings flag, so a pattern-only run never
    // reached CellManager at all.
    std::atomic<bool> oasisRequested{ false };
    void setOasisRequested(bool v) { oasisRequested.store(v, std::memory_order_relaxed); }
    void setOverBudgetFix(bool v) { overBudgetFix.store(v, std::memory_order_relaxed); }

    // True when a static masking pattern is selected. The extractor consults this so
    // it calls skipCell for pattern-only runs, which do NOT set System.enableOasis.
    bool maskPatternActive() const
    { return maskPattern.load(std::memory_order_relaxed) != MASK_PATTERN_OFF; }

    void configureMaskPattern(int pattern, int seed, int densityPct)
    {
        maskPattern.store(pattern, std::memory_order_relaxed);
        maskRandomSeed.store(seed, std::memory_order_relaxed);
        maskDensityPct.store(densityPct, std::memory_order_relaxed);
    }
    int  getMaskPattern() const { return maskPattern.load(std::memory_order_relaxed); }

    // Increment the frame's elapsed cells
    void incrementCell();

    // Skip Cell? 
    bool skipCell(const feature_extraction_state_t&);

    // Signal the end of a frame and reset elapsed cells, and actual time to do frame
    void endFrame(const double&, double);

    // Calculate average cells per frame
    double getAverageCellsPerFrame() const;

    // Debug print
    void printStats(const double&, const double&) const;

    // Delete copy constructor and assignment operator to enforce singleton pattern
    CellManager(const CellManager&) = delete;
    CellManager& operator=(const CellManager&) = delete;

private:
    // Only this class be making and destroying instances
    CellManager() = default;
    ~CellManager() = default;
};

} // namespace ORB_SLAM3
