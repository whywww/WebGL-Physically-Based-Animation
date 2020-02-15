// Force Types
F_NONE = 0;     // Non-existent force: ignore this CForcer object
F_MOUSE = 1;    // Spring-like connection to the mouse cursor;
F_GRAV_E = 2;   // Earth-gravity: pulls all particles 'downward'.
F_DRAG = 3;     // Viscous drag -- proportional to neg. velocity.
F_SPRING = 4;   // ties together 2 particles; distance sets force
F_SPRING_SNAKE = 5;
F_SPRING_TET = 6;
F_WIND = 7;     // Blowing-wind-like force-field;fcn of 3D position
F_TORNADO = 8;
F_FLOCK = 9;
F_MAXKINDS = 10; // 'max' is always the LAST name in our list;

class CForcer {
    forceType;  // sets the kind of force this object describes

    // F_GRAV_E  Earth Gravity variables........................................
    gravConst = 9.832;  // Gravitational constant
    downDir = new Float32Array([0.0, 0.0, -1.0]);  // the 'down' direction vector for gravity.

    // F_DRAG Viscous Drag Variables............................................
    K_drag = 0.985;  // force = -velocity*K_drag.
    
    // F_SPRING Single Spring variables
    K_spring = 3.0; // Spring stiffness
    K_springdamp = 0.1; // Spring damping
    K_springlen = 0.0;  // Spring rest length
    fixedPoint = [0.0, 0.0, 0.0];  //  Endpoint of spring that's fixed

    // Wind
    D_wind = 1; 
    v_wind = [1.0, 0.0, 0.0];

    // Tornado
    T_strength = 1;
    T_center = [0,0,0];

    // Flock
    neighborInnerR = 0.05;  // neighbour inner radius of a particle
    neighborOuterR = 0.2;  // neighbour out radius of a particle

    constructor(forceType){
        this.forceType = forceType;
    }
}