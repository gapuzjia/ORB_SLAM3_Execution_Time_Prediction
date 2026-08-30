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
    std::atomic<int> frame_budget;
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
