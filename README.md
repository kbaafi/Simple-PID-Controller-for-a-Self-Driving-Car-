# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Running the application
./pid expects three input variables ,Kp,Ki and Kd in that order.
* Ki is the proportionality constant
* Ki is the integral constant and 
* Kd id the differential constant of the PID controller

## Optimal values for PID
the best values for PID in that order are 
* Kp = -0.08
* Ki = 0
* Kd = -2

## Reflections
### Kp
When Kp is high say around (-0.5) I observe that the the vehicle takes longer to converge to a higher stability state. This usually happens in curves. At good values for Ki and Kd, even if the vehicle does not go off track, the vehicle wanders about a lot before settling.

When Kp is low ie close to 0, the vehicle slowly moves to veers away from the center and eventually ends up off the track. This happens even under optimal values of Ki and Kd

### Ki
When Ki is high the vehicle has a propensity to oscillate and veers off the track eventually. Even values of -0.001 causes the vehicle to veer off. Setting the value to 0 causes the vehicle to move in a more stable manner.

### Kd
Under optimal Ki and Kp values, When Kd is low say, between -0.1 and -0.5, the car finds it difficult to converge to a stable state when the cross track error becomes high. The Kd value influence how quickly the vehicle can return to stability, so when Kd is low, the vehicle will wander around before settling down. This is particularly prevalent in sharp curves or when the car moves from  a straight line into a curve. When the Kd is very high the vehicle becomes very averse to high cross track error values. It tries to correct this using sharp steering, the vehicle stays on the track  but in corners the car steers erratically.

## How the P,I,D coefficients were chosen
The coefficients were determined manually starting from -0.25, 0, and -0.25 for P,I and D respectively and manually examining the effect of the values on the stability of the car.

It must be noted that the throttle value was reduced from 0.3 to 0.2 to make the car more stable. A better solution will be to create new PID for controlling the speed or using the twiddle algorithm at the end of iteration.



