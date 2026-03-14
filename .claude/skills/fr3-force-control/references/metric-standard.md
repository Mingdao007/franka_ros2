# Metric Standard (Force Control)

## Policy
- Deprecated: `MSE`
- Required: `RMSE`
- Recommended normalized metrics:
  - `NRMSE_target = RMSE / |F_target|`
  - `NRMSE_range = RMSE / (F_max - F_min)`

## Why
- RMSE keeps physical units (N), so control quality is directly interpretable.
- NRMSE makes cross-task comparison easier when target magnitudes differ.

## Reporting format
Always include:
1. `RMSE (N)`  
2. `NRMSE_target (%)`  
3. `NRMSE_range (%)` (or `N/A` when force range is zero)
