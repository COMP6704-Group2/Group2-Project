

# Microgrid Energy Management with LP/QP (MATLAB)

This repository contains MATLAB implementations of a simple microgrid energy management system (EMS) with photovoltaic (PV), wind power, and a battery energy storage system (BESS). The same EMS model is solved using different linear and quadratic programming algorithms in `linprog` and `quadprog` to compare solver behavior, solution quality, and runtime.

The optimization horizon is 24 hours with 1-hour time steps. At each time step, the controller decides how much power to buy/sell from the grid and how to charge/discharge the BESS, given fixed PV/wind availability and load profiles.

---

## Problem Formulation

All scripts use the same decision variables (PV and wind are treated as given profiles and are not part of the optimization vector).

The decision vector is:

* (P_\text{buy}(1:T)): grid import power
* (P_\text{sell}(1:T)): grid export power
* (P_\text{ch}(1:T)): battery charging power
* (P_\text{dis}(1:T)): battery discharging power
* (E(1:T)): battery state-of-charge (SoC)

The objective is to minimize the total operating cost over the 24-hour horizon:

* Cost of buying electricity from the grid at price (\pi_\text{buy})
* Revenue (negative cost) from selling electricity at price (\pi_\text{sell} < \pi_\text{buy})
* Optional penalty on battery charging/discharging to reflect degradation cost

Subject to:

* Power balance at each hour (PV and wind are must-take renewable generation)
* BESS dynamics and SoC bounds
* Terminal SoC constraint (E_T = E_0)
* Power and energy bounds for grid and BESS

All scripts share the same model; only the solver and algorithm settings differ.

---

## Repository Structure

```text
.
├── qp_activeset.m        # Strongly convex QP solved by quadprog (active-set)
├── qp_interiorpoint.m    # Strongly convex QP solved by quadprog (interior-point-convex)
├── activeset.m           # LP solved by quadprog (active-set) in QP form (H = 0)
├── interiorpoint.m       # LP solved by linprog (interior-point)
├── dualsimplex.m         # LP solved by linprog (dual-simplex)
└── README.md             # Project documentation
```

> If your files are currently `.txt`, rename them to `.m` or update the filenames above accordingly.

---

## Requirements

* MATLAB (R2019b or later recommended)
* Optimization Toolbox:

  * `linprog` (for LP with interior-point and dual-simplex)
  * `quadprog` (for LP-in-QP form and strongly convex QP)

No external toolboxes or data files are required; all PV, wind, and load profiles are defined inside each script.

---

## How to Run

1. Clone or download this repository.

2. Open MATLAB and add the project folder to the MATLAB path.

3. In the MATLAB Command Window, run any of the following scripts:

   ```matlab
   % Linear program with linprog (interior-point)
   interiorpoint

   % Linear program with linprog (dual-simplex)
   dualsimplex

   % Linear program in QP form with quadprog (active-set)
   activeset

   % Strongly convex QP with quadprog (active-set)
   qp_activeset

   % Strongly convex QP with quadprog (interior-point-convex)
   qp_interiorpoint
   ```

4. After execution, each script:

   * Prints the optimal operating cost and total solver time in the Command Window.
   * Generates three figures summarizing the solution.

---

## Scripts Description

### `interiorpoint.m`

Solves the EMS as a linear program using `linprog` with the **interior-point** algorithm.

* Pure LP formulation with linear objective `f' * x`.

* Uses options similar to:

  ```matlab
  opts = optimoptions('linprog', ...
      'Algorithm','interior-point', ...
      'Display','iter');
  ```

* Generates three plots:

  1. PV, wind, and load profiles.
  2. Power balance (contributions from PV, wind, BESS, and grid).
  3. BESS SoC and charge/discharge power.

Use this script as the baseline LP formulation to observe the typical behavior of an interior-point solver on the microgrid EMS.

---

### `dualsimplex.m`

Solves the same LP with `linprog` using the **dual-simplex** algorithm.

* Same model, decision variables, and constraints as `interiorpoint.m`.

* Only changes the solver algorithm, e.g.:

  ```matlab
  opts = optimoptions('linprog', ...
      'Algorithm','dual-simplex', ...
      'Display','iter');
  ```

* Useful for:

  * Comparing runtime with the interior-point method.
  * Inspecting different iteration logs for the same EMS problem.

---

### `activeset.m`

Solves the LP using `quadprog` in QP form with the **active-set** algorithm (Hessian = 0, i.e., an LP encoded as a QP).

* Encodes the LP as:

  ```matlab
  H = sparse(n, n);   % zero Hessian for LP
  ```

* Builds the same equality and inequality constraints as the LP scripts.

* Constructs a **feasible initial point** `x0` based on the net load profile, with SoC initialized to `E0`.

* Calls:

  ```matlab
  [x, fval, exitflag, output] = quadprog(H, f, A, b, Aeq, beq, lb, ub, x0, opts);
  ```

This script illustrates how to reuse `quadprog`’s active-set implementation for linear programs and shows how the choice of initial active set can affect convergence.

---

### `qp_activeset.m`

Solves a **strongly convex QP** version of the EMS problem using `quadprog` with the **active-set** algorithm.

* Adds a small quadratic regularization term to the objective:

  [
  \min\ f^\top x + \frac{\varepsilon}{2}|x|^2
  ]

  where (\varepsilon \ll 1), scaled to the cost magnitude.

* The Hessian becomes strictly positive definite:

  ```matlab
  eps_qp = 1e-6 * max([pi_buy, c_sto, 1]);
  H      = speye(n) * eps_qp;
  ```

* Keeps the EMS cost almost identical to the LP while:

  * Improving numerical stability.
  * Ensuring strict convexity and uniqueness of the solution (up to numerical precision).

* Uses a feasible initial point `x0` for the active-set method, similar to `activeset.m`.

This script demonstrates how a lightly regularized QP behaves compared to the pure LP, and highlights the effect of strict convexity on solver robustness.

---

### `qp_interiorpoint.m`

Solves the same **strongly convex QP** using `quadprog` with the **interior-point-convex** algorithm.

* Uses the same regularized objective and Hessian as in `qp_activeset.m`.

* Does **not** require an explicit initial point `x0`:

  ```matlab
  opts = optimoptions('quadprog', ...
      'Algorithm','interior-point-convex', ...
      'Display','iter');
  [x, fval, exitflag, output] = quadprog(H, f, A, b, Aeq, beq, lb, ub, [], opts);
  ```

* Useful for:

  * Comparing active-set vs interior-point on the same QP.
  * Studying differences in convergence behavior and runtime.
  * Evaluating sensitivity to the regularization strength `eps_qp`.

---

## Output Figures

Each script produces the same three figures to facilitate visual comparison across solvers:

1. **PV, Wind, and Load**

   * Time series of PV output, wind output, and load over the 24-hour horizon.

2. **Power Balance**

   * Visualization of contributions from PV, wind, BESS charge/discharge, and grid import/export.
   * Overlaid with the load profile to verify that the power balance holds at every time step.

3. **BESS SoC and Charge/Discharge**

   * Line plot of battery state-of-charge (SoC) over time.
   * Bars or lines showing charge and discharge power at each time step.

These plots help confirm that:

* Power balance is satisfied at each hour.
* SoC remains within specified bounds and returns to the initial level at the end of the horizon.
* Different solvers produce consistent operational strategies and very similar total costs.

---

## Extending the Code

You can extend or adapt this project in several ways:

* Change PV, wind, and load profiles to test different scenarios.
* Modify electricity prices, BESS parameters, or the time horizon.
* Add new constraints (e.g., ramp limits, demand response, minimum up/down times).
* Introduce additional algorithms or solver configurations for comparison.
* Integrate real data from practical microgrid or energy management case studies.

---

