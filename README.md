# STUNT - ruSt daTaflow aUtoNomous piloT

STUNT is an experimental autonomous driving pipeline leveraging Zenoh Flow
for communication between the components


:warning: **This software is still in alpha status and should _not_ be used in production. Breaking changes are likely to happen and the API is not stable.**


---

STUNT mixes python components, getting information for the CARLA (v0.9.13) simulator, with
rust-based components doing the hard calculations.


All data exchanged between the components is serialized as JSON.