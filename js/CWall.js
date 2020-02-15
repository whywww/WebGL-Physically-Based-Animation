WTYPE_DEAD = 0;         // DEAD CONSTRAINT
WTYPE_GROUND = 1;       // y=0 ground-plane
WTYPE_XWALL_LO = 2;     // planar X wall; keeps particles >= xmin
WTYPE_XWALL_HI = 3;
WTYPE_YWALL_LO = 4;
WTYPE_YWALL_HI = 5;
WTYPE_ZWALL_LO = 6
WTYPE_ZWALL_HI = 7;
WTYPE_ANCHOR = 8;       // Lock one particle at location xpos,ypos,zpos
WTYPE_SEPERATE = 9;        // Connects 2 particles with fixed-length separation
WTYPE_TORNADO = 10;
WTYPE_FIRE = 11;
WTYPE_PBALL = 12;       // solid sphere centered at particle with index e0;
WTYPE_MAXVAR = 13;     

var Kbouncy = 1.0;  // Coeff. of restoration for constraint surfaces
var pBallRadius = 0.1;  // constraint ball radius
var pBallCenter = [0.0, 0.0, 1.0];  // constraint ball center
var ballRadius = 0.05;  // radius of particle balls


class CWall {
    wallType;
    Kbouncy = 1.0;

    xpos = 0.0; ypos = ballRadius; zpos = ballRadius; 
    xmin = -1.0+ballRadius; xmax = 1.0-ballRadius; ymin = -2.0+ballRadius; ymax = 1-ballRadius; zmin = 0.0+ballRadius; zmax = 1.8-ballRadius;
    wallSize = 0;   // limit on size of WTYPE_WALL constraints; wall
                // extends outwards +/-wallSize from its starting
                // point at xpos,ypos,zpos.  If wallSize <= 0.0,
                // then wall size is unlimited.
    nx; ny; nz;

    constructor(wallType){
        this.wallType = wallType;
    }
}