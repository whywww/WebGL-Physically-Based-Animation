// Particle Types
PTYPE_DEAD = 0;   // DEAD PARTICLE
PTYPE_ALIVE = 1;  // 'default' particle
PTYPE_BALL = 2;   // small bouncy round shiny sphere particle
PTYPE_MAXVAR = 3;

// Particle Attributes
PART_MASS = 0;
PART_XPOS = 1;
PART_YPOS = 2;
PART_ZPOS = 3;
PART_WPOS = 4;
PART_XVEL = 5;
PART_YVEL = 6;
PART_ZVEL = 7;
PART_X_FTOT = 8;
PART_Y_FTOT = 9;
PART_Z_FTOT = 10;
PART_SIZE = 11;
PART_MAXAGE = 12;
PART_AGE = 13;  // # of frame-times since creation/initialization
PART_R = 14;
PART_G = 15;
PART_B = 16;
PART_A = 17;
PART_MAXVAR = 18;

class CPart{
    partType;   // Particle type; not required, but a) helps you
                // identify the intended purpose of each particle,
                // and b) gives you an easy way to enable/disable
                // each particle:
                //  partType  >0 == active particle; use it! the
                //                 value describes its use
                //  partType ==0 == 'dead' particle, abandoned,
                //                  ignored, available for re-use.
                //  partType  <0 == temporarily disabled 'frozen';
                //                  to re-enable this particle,
                //                  set partType = -partType;
    val = new Float32Array(PART_MAXVAR);
}