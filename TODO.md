# World management
- Updated through `PosesStamped` message (currently easiest, might go for more efficient
  solution later).

# Joint force / velocity instability problem
## The problem
Joints reach ridiculously high velocities making the simulation unstable.

## Some info
A servo has both a velocity rating and a torque rating. Both these numbers
are maxima, i.e. the servo cannot go faster than the velocity rating and
cannot exert more torque than the torque rating.  

## The cause
In the simulation, we either add a torque to a joint, or we set its
velocity explicitly. Doing the latter ignores all physics and sets
the velocity of the bodies regardless of the maximum forces the joint
motor can exert. For realism I would thus rather avoid this. Doing
the former will prevent any forces from being applied to the joint
once it moves on or above the allowed velocity. This does not solve
the problem however, since as it would turn out the maximum torque
of our servo's can make a joint with little load (i.e. little resistance)
accelerate to very large velocities in one timestep, after which
not allowing any forces is simply too late (it's alread moving insanely fast).

## Some numbers
At a 1/1000 s timestep update rate, a rotator pointing up without any
attachment (I'm assuming this is about the "lightest" case) will move
to the maximum velocity with a force of only 0.005 times the currently
defined effort limit. Apply any more, and the velocity will be too
large without it being possible to do anything about it.
Let's make a table of how much maximum rotational
torque influences angular velocity based on the weight of the attachment:

Block weight    1dt velocity
1000            1.2
 500            2.4

## Possible solutions
1. Set the velocities directly, like it happens in RoboGen. This means bypassing
   the main Gazebo APIs and talking to ODE directly (or rather call ODE specific
   functions through the Gazebo API), which is undesirable for obvious reasons. 
   It is however very simple to do, ODE will enforce the maximum forces as
   constraints in calculating the angular velocity in the next timestep.
2. Fine-tune the PID-controller to make sure this never happens. The problem
   is that we do not know what type of force we need to reach a certain speed,
   but this is of course what a PID-controller is for. When the joint load is
   large, we might still have to apply maximum force to gain movement. I'm hoping
   that with some careful tuning I can use the integral term for this. If
   that works the force will build up until there's enough force to move
   the motor. After this it will of course continue building up until the
   desired position is reached so it will lead to overshooting. I might then
   have to prevent this with the derivative term, in order to have the
   error buildup effect decrease when the error decreases.
3. Maintain some "correcting factor" within the motor, i.e. check over time
   how much velocity can be gained by which percentage of the relative torque,
   and adapt the maximum motor output based on that. Should start with
   a small value. This seems like a rather messy solution, it would be a sort
   of PID controller for the PID controller.

I'm currently going for (1) since it's so very easy to implement. 