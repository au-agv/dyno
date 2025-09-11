# Extending functionalities

## Using your vehicle models

There are three options for vehicle models in DYNO:

- Use one of the vehicle models directly available in DYNO (currently only the OLAV model).

- Use one of the JSON vehicle models from Project Chrono

- Use a custom vehicle model:

### Project Chrono vehicle models

### Custom vehicle models

If you would like to implement your own vehicle model:

* Derive from the class DYNO::Models::WheeledVehicle for wheeled vehicle models or DYNO::Models::TrackedVehicle for tracked vehicle models.

* Implement all possible virtual methods. Not all methods are used in all scenarios, hence depending on your application you may not have to implement all methods. In the case of missing implementations, we recommend throwing a DYNO::NotImplemented exception.
