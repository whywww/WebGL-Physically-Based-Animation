WTYPE_DEAD = 0;         // DEAD CONSTRAINT
WTYPE_GROUND = 1;       // y=0 ground-plane
WTYPE_XWALL_LO = 2;     // planar X wall; keeps particles >= xmin
WTYPE_XWALL_HI = 3;
WTYPE_YWALL_LO = 4;
WTYPE_YWALL_HI = 5;
WTYPE_ZWALL_LO = 6
WTYPE_ZWALL_HI = 7;
WTYPE_ANCHOR = 8;       // Lock one particle at location xpos,ypos,zpos
WTYPE_STICK = 9;
WTYPE_AGE = 10;
WTYPE_MAXVAR = 11;     

var ballRadius = 0.07;  // radius of ball

class CWall {
    wallType;
    Kbouncy = 1.0;  // Coeff. of restoration for constraint surfaces
    xpos = 0.0; ypos = ballRadius; zpos = ballRadius; 
    xmin = -1.0+ballRadius; xmax = 1.0-ballRadius; ymin = -2.0+ballRadius; ymax = 1-ballRadius; zmin = 0.0+ballRadius; zmax = 1.8-ballRadius;
    wallSize = 0;   // limit on size of WTYPE_WALL constraints; wall
                // extends outwards +/-wallSize from its starting
                // point at xpos,ypos,zpos.  If wallSize <= 0.0,
                // then wall size is unlimited.
    nx; ny; nz;
    partSetSize = 0;    // How many particles affected by this CWall object.
    //                 // -- 0 == 'all particles'
    //                 // -- 1 == one particle; e0 holds its index number.
    //                 // -- 2 == two particles; e0,e1 holds index numbers.
    // e0; e1;  // Particle-index #s if we need only 1 or 2.

    constructor(wallType){
        this.wallType = wallType;
    }
}