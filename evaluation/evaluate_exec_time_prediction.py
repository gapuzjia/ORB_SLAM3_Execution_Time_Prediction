"""
evaluate_exec_time_prediction.py –  actual vs predicted time per cell evaluator

Outputs:
    

"""
# --------------------------------------------------------------------------
import os
import numpy as np
import pandas as pd
# --------------------------------------------------------------------------

base_dir = os.path.join("..", "ExecTimeEvalResults")

# iterate through each folder in ExecTimeEvalResults
for folder in os.listdir(base_dir):
    folder_path = os.path.join(base_dir, folder)

    # declare results file path
    results_file = os.path.join(folder_path, f"{folder}results.txt")

    results = []

    for file in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file)
        df = pd.read_csv(file_path, low_memory=False)

        # ----- compute metrics

        # convert to numbers
        df['actual_ms'] = pd.to_numeric(df['actual_ms'], errors='coerce')
        df['predicted_ms'] = pd.to_numeric(df['predicted_ms'], errors='coerce')
        df['actual_cells'] = pd.to_numeric(df['actual_cells'], errors='coerce')
        df['avg_cells_per_frame'] = pd.to_numeric(df['avg_cells_per_frame'], errors='coerce')
        df['skipped'] = pd.to_numeric(df['skipped'], errors='coerce')

        df["actual_tpc"] = (
            df["actual_ms"] / df["actual_cells"]).replace([np.inf, -np.inf], 0).fillna(0)
        df['pred_tpc'] = (
            df['predicted_ms'] / df['avg_cells_per_frame']).replace([np.inf, -np.inf], 0).fillna(0)
        df['residuals_squared'] = np.square(df['actual_tpc'] - df['pred_tpc'])
        rmse = np.sqrt(np.sum(df['residuals_squared']) / len(df))
        skipped_frames = df['skipped'].sum()
        avg_tpc_actual = df['actual_tpc'].mean()
        avg_tpc_pred = df['pred_tpc'].mean()

        # ----- store results
        with open(results_file, 'a') as f:
            f.write(f"{file}\t: {rmse}\t{skipped_frames}\t {avg_tpc_actual}\t {avg_tpc_pred}\n")

        #----- print results
        print(f"Processed {file} in {folder}, RMSE: {rmse}, Skipped Frames: {skipped_frames}, Mean TPC act: {avg_tpc_actual}, Mean TPC pred:{avg_tpc_pred}")
