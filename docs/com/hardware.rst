Hardware
========

1.1 Dimensions
--------------

- 52x52x80 cm (just frame)
- 52x52x84 cm (including wheels)

Dimensions decided based on the requirements that MOMO has to fit inside lifts, and pass through doors.

The ratio of 52 cm : 80 cm of the frame is to improve the aesthetics of the MOMO, by observing the golden ratio.

1.2 Specifications
------------------

Side panels are tapered on one end to allow for a 270 degree field-of-view to ensure unobstructed LIDAR operation. Components on the bottom are shifted to the rear in addition to tapered side panels to allow for unobstructed view for the LIDAR.

MOMO uses rear wheel drive to ensure maximum stability when driving forward. Front wheel drive systems are inherently unstable due to fluttering of the castor wheel - undesirable oscillations during autonomous navigation (this was shown empirically as the original design was a front wheel drive).

This allows for the robot to deal with any sudden loss of front traction as well as traverse ramps.

**Design Payload Weight:** 20 kg
In actuality, MOMO can take the weight of 1 person (60+ kg) when both motors are moving. (However, it would be unable to perform pivot turns.)

Some quirks in selection and design
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2040 Extrusions are utilised to allow for greater loading of frame by increasing the amount of material vertically. This increases the maximum strain which the frame can withstand.

4 corner pillars of MOMO comprise of L-shaped 4040s to prevent horizontal warping of the frame.

Reason for more extrusions in front:
- Expose more of the structure to allow lidar to function
- Would have to pay attention to how frame is supported

Reason to do rear wheel drive
- Enable robot to handle uneven ground:

  - Middle wheel drive - on uneven ground, castors lift

  - Front wheel drive - spirals out of control, oscillates during navigation

Motors are mounted with 2 thick aluminum plates
Motors that slide in from the side allow for ease of maintenance

Bottom layer braced with L shaped extrusions to stop battery from moving too much, as well as to provide a sturdier frame.

**Momo as 4 seperate layers:**

1. Motor and Battery
1.5. Lidar
2. Electronics - driver, relays, mcu, esc, all else
2.5. Screen
3. Computer
3.5. E-stop
4. Any thing else to be added

Castors chosen so as not to leave marks (stay away from rubber wheels -  first waiterbot used rubber castor wheels which left black marks at Fablab when navigating )

After a long day of running Momobot, there is a need to check gussets as they may loosen. Very important to check the bottom as there are over 20 gussets at the bottom securing the batteries in place.

Can be tipped without problem as frame is sturdy

Good design:
Low CG - able to stabilize quicky after tilting

CG weight
Rear wheel drive - all weight on rear wheel, when accelerate does wheelie, so weight was shifted forward

1.3 Gotchas, Hacky Stuff and Things to Take Note Of
---------------------------------------------------

Take note:
- Screws to motor -able to come out with vibrations:

  - Remember to check gusset bolt tightness if MOMO has been operated with many vibrations

- Caster wheels: Have to use washers to compansate
- Rounded Motor mount bolts:

    - Motor mount mounted to extrusion with 6 bolts - some of the bolts are rounded and cannot be removed
    
- 2nd layer - to screen - the screen mount was not designed for Momo
- Arm not compatible with back plate, backplate not compatible with screen
- Bought mount, Could not extend to extrusions
- Arm to mount screen forced upwards to fit the screen inside. This however, locks the screen in place.
- Castor wheel mountings - Castor wheel mountings too big for screw.
- Screw to giant washer to washer to attach the caster wheels at the bottom.

1.4 To Dos
----------

1. Redo Lidar Mount - side holder tolerance w
2. Implement a guard for the front LiDAR
3. Implement Rear LiDAR
4. Implement system to improve ease of removing back panel (i.e. magnets, hooks), current back panel is screwed on by 6x rhombus nuts. [In-progress]
5. Implement a charging port for both 55Ah Batteries and 7Ah Batteries
