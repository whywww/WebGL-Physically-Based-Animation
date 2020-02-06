// Particle Types
PTYPE_DEAD = 0;   // DEAD PARTICLE
PTYPE_ALIVE = 1;  // 'default' particle
PTYPE_BALL = 2;   // small bouncy round shiny sphere particle
PTYPE_MAXVAR = 3;

// Particle Attributes
PTYPE_DEAD = 0;   // 0: DEAD PARTICLE; 1: ALIVE 
PART_MASS = 1;
PART_XPOS = 2;
PART_YPOS = 3;
PART_ZPOS = 4;
PART_WPOS = 5;
PART_XVEL = 6;
PART_YVEL = 7;
PART_ZVEL = 8;
PART_X_FTOT = 9;
PART_Y_FTOT = 10;
PART_Z_FTOT = 11;
PART_SIZE = 12;
PART_MAXAGE = 13;
PART_AGE = 14;  // # of frame-times since creation/initialization
PART_R = 15;
PART_G = 16;
PART_B = 17;
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