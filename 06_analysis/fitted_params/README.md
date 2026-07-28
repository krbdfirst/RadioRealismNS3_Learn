# Fitted parameters

Stage-5 output: the coefficients produced by fitting the surrogates to the ns-3
teacher, before they are packaged for the run time. Each file records the fitted
values and the functional form they belong to, so a coefficient can be traced
back to the equation it came from.

These are the fit results. What the simulation actually loads at run time is the
packaged bundle in `../../01_omnet_project/model_coefficients/ns3learn_runtime/`;
`export_ns3learn_runtime.py` and `export_cascade_coeffs.py` do the packaging.

| File | Produced by | Fits |
|---|---|---|
| `surrogate_params.csv` | `fit_ns3_surrogate.py` | The first-generation scalar surrogate: half-duplex fraction, capture fraction, and the interference term `I_db = a·log10(1+n)`. Superseded by the cascade for the reported results, kept because the earlier phases are reported against it. |
| `collision_fit_params.csv` | `fit_ns3_surrogate.py` | Collision against neighbour count as a 3-parameter logistic, `L/(1+exp(-k(n-x0)))`. The cascade's `col_c*` quadratic form replaced this. |
| `load_collision_params.csv` | `fit_contention_load.py` | Collision as a function of **offered load** rather than vehicle count, `sigmoid(c0+c1·log10(n·Hz))`. Tests whether load alone explains contention. |
| `product_collision_params.csv` | `fit_contention_product.py` | Per-interferer product form: each in-range interferer contributes a bounded factor `q` that is 2-D logistic in its rate and the local density; survival is the product over interferers. |
| `attacker_factor_params.csv` | `fit_attacker_factor.py` | The deployed high-rate-interferer factor `q_a`. These four values (`qa_b0..b3`) are the ones exported into `cascade_params.csv` and read by the run time. |

The three contention files record the progression described in Phase 16 of
`docs/NS3_TO_OMNET_DISTILLATION.md`: a lumped load model, then a per-interferer
product model, then the hybrid that keeps the validated baseline `base(n)` fixed
and admits an interferer only as one bounded factor. Only the last is deployed.

`load_collision_params.csv` and `product_collision_params.csv` are retained
because the held-out test that rejected the lumped model in favour of the product
form is part of the argument, not a discarded intermediate: concentrated load
collides *less* than the same load spread over many nodes, which the lumped model
over-predicts.
