# Bumblebee FastAerialStrike dependency for Volt

Volt's aerial routine now uses the RLUtilities native `Aerial` controller from RLBotPack's Botimus & Bumblebee FastAerialStrike implementation.

## Required native package

Copy the **entire** folder:

`RLBotPack/Botimus&Bumblebee/rlutilities`

from your local RLBotPack checkout into Volt's project root, so the result is:

`Volt/rlutilities/__init__.py`
`Volt/rlutilities/rlutilities.cp311-win_amd64.pyd`
`Volt/rlutilities/assets/...`

The CPython 3.11 Windows x64 binary matches Volt's Python 3.11.9 environment. Do not copy the CPython 3.7 binary.

The native package includes the compiled extension and mesh/model assets. The Python aerial adapter imports it lazily; if it is missing, Volt logs `[BUMBLEBEE AERIAL UNAVAILABLE]` and falls back to its previous aerial controller instead of crashing.

## Boost handling

The original Bumblebee `AerialStrike.simulate_flight` resets simulated boost to 100 on every frame because RLUtilities' simulated car does not deplete boost itself. Volt's adapter deliberately does **not** do that. It tracks the simulated boost pool and subtracts 33.333 boost/s whenever the simulated controller boosts. It also preserves the post-launch boost reserve produced by Volt's shot finder while driving to the takeoff position.

## What is retained from Volt

- VirxERLU / C shot finder and selected intercept
- Volt's goal aim and surface contact-point selection
- Existing boost-budget calculation
- Wall-aerial detection and the rest of the strategy / teamplay code

The in-flight orientation, double-jump aerial controller, and simulated takeoff reachability are provided by RLUtilities, following Bumblebee's FastAerialStrike approach.

## Runtime check

After copying the dependency, start Volt and check the CMD output for:

- `[BUMBLEBEE FAST AERIAL] launch`
- `[BUMBLEBEE FAST AERIAL] flying`

If you see `[BUMBLEBEE AERIAL UNAVAILABLE]`, the `rlutilities` folder is missing, in the wrong location, or has the wrong Python ABI.
