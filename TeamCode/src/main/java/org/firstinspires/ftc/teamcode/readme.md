## Organization

The files in this module are organized into 3 sub-modules: ``opmodes``, ``subsystems``, and ``util``.

The ``opmodes`` module contains the various teleop and autonomous opmodes that can be
run from the driver station.

The ``subsystems`` module contains all hardware and software subsystems that control
the robot. These subsystems include the drivetrain, localization, vision, and any subsystems
such as arms or lifts.

The ``util`` module contains any utility classes that will be used by the other two modules.
These utility classes are things such as PID controllers and Math functions used across
multiple files.

## Naming Conventions
This repository will follow the [official naming conventions](https://www.oracle.com/java/technologies/javase/codeconventions-namingconventions.html).
