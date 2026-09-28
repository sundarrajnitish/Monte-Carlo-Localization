# Legacy code (2023)

The first version of this project, kept **unmodified** for reference. It does
not produce a working filter; see "What changed" in the top-level README (or
chapter 8 of the website) for the list of problems and how each was fixed in
`python/` and `matlab/`.

* `matlab/` - the original MATLAB particle filter, sensor models and map viewer.
  `EstimatedParameters.txt` holds the original noise parameters.
* `hallway_1d/` - the 1-D "corridor with three doors" warm-up example
  (doors at 3, 5 and 10 m) that the website's 1-D widget reproduces.
