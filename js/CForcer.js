// Force Types
F_NONE = 0;     // Non-existent force: ignore this CForcer object
F_MOUSE = 1;    // Spring-like connection to the mouse cursor;
F_GRAV_E = 2;   // Earth-gravity: pulls all particles 'downward'.
F_DRAG = 3;     // Viscous drag -- proportional to neg. velocity.
F_SPRING = 4;   // ties together 2 particles; distance sets force
F_MAXKINDS = 5; // 'max' is always the LAST name in our list;

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

    constructor(forceType){
        this.forceType = forceType;
    }
}