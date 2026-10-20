# Third-party models

| Folder | Model | Source | Version | License | Changes |
|---|---|---|---|---|---|
| `rovi_panda_fuel_v5/` | Franka Emika Panda with position controllers and PBR materials, by OpenRobotics (initially from AndrejOrsula/panda) | Gazebo Fuel: `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Panda with Ignition position controller model` | 5 (zip SHA-256 `86f3a9b45ee2a057ce3ff43f624fceb113e1554f6f2fb762dc6b52d85d497298`) | Creative Commons Attribution 4.0 International | The 11 `<environment_map>` elements, which point to a remote texture (`https://fuel.ignitionrobotics.org/.../DepotEnvMap.dds`) outside the pinned archive, are removed so that rendering never downloads unpinned content; metal surfaces lose that reflection map. Nothing else is changed. `scripts/fetch_models.sh` applies the change and checks the result against a pinned digest of the whole tree (`48aae2c8…3908`). The model is not committed. |
