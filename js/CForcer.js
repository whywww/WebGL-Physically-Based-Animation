// Force Types
F_NONE = 0;     // Non-existent force: ignore this CForcer object
F_MOUSE = 1;    // Spring-like connection to the mouse cursor;
F_GRAV_E = 2;   // Earth-gravity: pulls all particles 'downward'.
F_DRAG = 3;     // Viscous drag -- proportional to neg. velocity.
F_MAXKINDS = 4; // 'max' is always the LAST name in our list;

class CForcer {
    forceType;  // sets the kind of force this object describes

    // F_GRAV_E  Earth Gravity variables........................................
    gravConst = 9.832;  // Gravitational constant
    downDir = new Float32Array([0.0, 0.0, -1.0]);  // the 'down' direction vector for gravity.

    // F_DRAG Viscous Drag Variables............................................
    K_drag = 0.985;  // force = -velocity*K_drag.

    constructor(forceType){
        this.forceType = forceType;
    }
}