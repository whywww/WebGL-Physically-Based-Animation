SOLV_EULER = 0;
SOLV_IMPLICIT = 1;
isFountain = 0;
class CPartSys {
    //-------State Vectors-----------------------!
    partCount = 0;    // # of particles held in each state vector. (if <=0, state vectors set to NULL)
    S0; S0dot;  // state vector holding CURRENT state of particles, and its time derivative s0dot.
    S1; S1dot;  // NEXT state, and its time-derivative.
    SM; SMdot;  // midpoint state vector and its time-deriv.
    
    //-------Force-List Vector(s)----------------
    forcerCount = 0;  // # of forcer-making objects held in the
                  // dyn. alloc'd list at pF0 (if needed, in
                  // pF1, pFM, pF0dot,pF1dot,pFMdot as well).
    F0;          // f0; forcer-vector-- dyn. alloc'd list of all CURRENT force-making objects,

    //-------Wall-Vectors(s)---------------
    wallCount = 0;    // # of constraint-making objects (CWall obj)
                  // held in the dyn. alloc'd list at pC0.
    C0;          // c0; constraint-vector--dyn. alloc'd list
                  // of all CURRENT constraint-making objects
    INIT_VEL =  0.15 * 60.0;
    
    /**
     * argument selects among several different kinds of particle systems and initial conditions.
     * @param {int} partCount 
     * @param {Array} forces 
     * @param {Array} walls 
     */
    init(partCount, forces, walls){
        this.partCount = partCount;
        this.forcerCount = forces.length;
        this.wallCount = walls.length;

        // initialize s0 for all particles
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);  // Float32Array or Array of obj???????
        
        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){  // init mass
            this.S0[j + PTYPE_DEAD] = 1;
            this.S0[j + PART_MASS] = 1.0;
            this.S0[j + PART_XPOS] = 1.5*(Math.random()-0.5);
            this.S0[j + PART_YPOS] =  1.5*(Math.random()-0.5);
            this.S0[j + PART_ZPOS] =  0.0;
            this.S0[j + PART_WPOS] =  1.0;
            this.S0[j + PART_XVEL] =  0.0;
            this.S0[j + PART_YVEL] =  0.0;
            this.S0[j + PART_ZVEL] =  0.0;
            this.S0[j + PART_X_FTOT] =  0.0;
            this.S0[j + PART_Y_FTOT] =  0.0;
            this.S0[j + PART_Z_FTOT] =  0.0;
            this.S0[j + PART_AGE] =  30 + 100*Math.random();
            this.S0[j + PART_MASS_VEL] =  0.0;
            this.S0[j + PART_MASS_FTOT] =  0.0;
            this.S0[j + PART_R] =  1.0;
            this.S0[j + PART_G] =  1.0;
            this.S0[j + PART_B] =  1.0;
        }
 
        // initialize s1 as a copy from s0
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);  // Deep Copy. Check this!!!  Make undefind 0???
        for (i = 0; i < this.S0.length; i++){
            this.S1[i] = this.S0[i];
        }
        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot = JSON.parse(JSON.stringify(this.S0));
        this.setArr(this.S0dot, 0);  // Q: What about partType?
        
        this.solvType = SOLV_IMPLICIT;
        // initialize f0, the list of CForcer for s0
        this.F0 = new Array(); // Do we need single CPart/force object?
        for (var i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));
        }

        // initialize constraints c0 used to adjust s1
        this.C0 = new Array();
        for (var i = 0; i < this.wallCount; i++){
            this.C0.push(new CWall(walls[i]));
        }
    }


    /**
     * Calculate FTOT and update current state S
     * @param {Float32Array} S Current state vector
     * @param {Array} F Current Force List
     */
    applyAllForces(S, F){
        // Clear force accumulators for each particle
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){
            S[j + PART_X_FTOT] = 0.0;
            S[j + PART_Y_FTOT] = 0.0;
            S[j + PART_Z_FTOT] = 0.0;            
        }
        // Change mass here???
        

        // Step through the forcers. Accumulate all forces.
        for (j = 0; j < this.forcerCount; j++){
            switch(F[j].forceType){
                case -F_GRAV_E:  // disabled gravity. Do nothing.
                    break;
                case F_GRAV_E:  // Earth gravity.
                    for (i = 0; i < this.partCount; i++){  // For every particle
                        // if (S[i].partType <= 0){  // Dead particle
                        //     continue;
                        // }
                        var mag = S[i * PART_MAXVAR + PART_MASS] * F[j].gravConst;  // magnitude of gravity's force. F=ma
                        // make a vector: scale our unit vector in the 'down' direction:
                        S[i * PART_MAXVAR + PART_X_FTOT] += mag * F[j].downDir[0];
                        S[i * PART_MAXVAR + PART_Y_FTOT] += mag * F[j].downDir[1];
                        S[i * PART_MAXVAR + PART_Z_FTOT] += mag * F[j].downDir[2];
                    }
                    break;
                case -F_DRAG:  // disabled drag; do nothing;
                    break;
                case F_DRAG:  // viscous drag: force = -velocity*K_drag.
                    for(i = 0; i < this.partCount; i++){
                        // if(S[i].partType <= 0){  // skip 'dead' particles
                        //     continue; 
                        // }
                        // add to force-accumulator
                        S[i * PART_MAXVAR + PART_X_FTOT] +=  
                                        -F[j].K_drag * S[i * PART_MAXVAR + PART_XVEL];
                        S[i * PART_MAXVAR + PART_Y_FTOT] +=
                                        -F[j].K_drag * S[i * PART_MAXVAR + PART_YVEL];
                        S[i * PART_MAXVAR + PART_Z_FTOT] +=
                                        -F[j].K_drag * S[i * PART_MAXVAR + PART_ZVEL];
                    }
                    break;
                case F_MOUSE:
                    break;
                case F_NONE:
                    break;
                default:
                    break;
            }
        }
    }


    /**
     * Find sDotDest using Newton's Law
     * @param {Float32Array} S0dot Time-derivitave of state vector sNow
     * @param {Float32Array} S0 Current state vector
     */
    dotMaker(S0dot, S0){
        for (i = 0; i < this.partCount; i++){  // For every particle
            
            S0dot[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XVEL];
            S0dot[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YVEL];
            S0dot[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZVEL];
            S0dot[i * PART_MAXVAR + PART_WPOS] = 0.0;
            
            // Compute a=F/m as sdot velocity.
            S0dot[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_X_FTOT] / S0[i * PART_MAXVAR + PART_MASS];
            S0dot[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_Y_FTOT] / S0[i * PART_MAXVAR + PART_MASS];
            S0dot[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_Z_FTOT] / S0[i * PART_MAXVAR + PART_MASS];

        }
    }


    /**
     * Finds next state S1. Implicit.
     * @param {milliseconds} timeStep 
     * @param {Float32Array} S0 Current state vec
     * @param {Float32Array} S0dot time-derivitive of current state
     * @param {Float32Array} S1 Next state vec to be computed
     */
    solver(timeStep, S0, S0dot, S1){
        switch (this.solvType){
            case SOLV_EULER:
                break;
            case SOLV_IMPLICIT:
                for (i = 0; i < this.partCount; i++){
                    // IMPLICIT. Compute pos & vel for new state vector
                    S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + S0dot[i * PART_MAXVAR + PART_XVEL]* timeStep *0.001;
                    S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + S0dot[i * PART_MAXVAR + PART_YVEL]* timeStep *0.001;
                    S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + S0dot[i * PART_MAXVAR + PART_ZVEL]* timeStep *0.001;
                    S1[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + S1[i * PART_MAXVAR + PART_XVEL] * timeStep * 0.001;
                    S1[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + S1[i * PART_MAXVAR + PART_YVEL] * timeStep * 0.001;
                    S1[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + S1[i * PART_MAXVAR + PART_ZVEL] * timeStep * 0.001;
                }
                break;
        }
    }


    /**
     * Adjust S1 & S2 to satisfy rules of collisions
     * @param {Float32Array} S2 Curr state
     * @param {Float32Array} S1 Prev state
     * @param {Array} W List of Constraints
     */
    doConstraints(S2, S1, W){
        // step through constraints
        for (j = 0; j < this.wallCount; j++){
            if (W[j].partSetSize == 0){  // limit applied on all particles
                for (i = 0; i < this.partCount; i++){
                    switch(W[j].wallType){  
                        case WTYPE_GROUND:
                            if (S2[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                                S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                                S2[i * PART_MAXVAR + PART_ZVEL] = 0.985*S1[i * PART_MAXVAR + PART_ZVEL];
                                if (S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_YWALL_LO:
                            if (S2[i * PART_MAXVAR + PART_YPOS] < W[j].ymin && S2[i * PART_MAXVAR + PART_YVEL] < 0.0){  // collision
                                S2[i * PART_MAXVAR + PART_YPOS] = W[j].ymin;
                                S2[i * PART_MAXVAR + PART_YVEL] = 0.985*S1[i * PART_MAXVAR + PART_YVEL]; // Still apply drag??
                                if (S2[i * PART_MAXVAR + PART_YVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_YWALL_HI:
                            if (S2[i * PART_MAXVAR + PART_YPOS] > W[j].ymax && S2[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                S2[i * PART_MAXVAR + PART_YPOS] = W[j].ymax;
                                S2[i * PART_MAXVAR + PART_YVEL] = 0.985*S1[i * PART_MAXVAR + PART_YVEL];
                                if (S2[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_XWALL_LO:
                            if (S2[i * PART_MAXVAR + PART_XPOS] < W[j].xmin && S2[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                S2[i * PART_MAXVAR + PART_XPOS] = W[j].xmin;
                                S2[i * PART_MAXVAR + PART_XVEL] = 0.985*S1[i * PART_MAXVAR + PART_XVEL];
                                if (S2[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_XWALL_HI:
                            if (S2[i * PART_MAXVAR + PART_XPOS] > W[j].xmax && S2[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                S2[i * PART_MAXVAR + PART_XPOS] = W[j].xmax;
                                S2[i * PART_MAXVAR + PART_XVEL] = 0.985*S1[i * PART_MAXVAR + PART_XVEL];
                                if (S2[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_ZWALL_LO:
                            if (S2[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                                S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                                S2[i * PART_MAXVAR + PART_ZVEL] = 0.985*S1[i * PART_MAXVAR + PART_ZVEL];
                                if (S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;

                        case WTYPE_ZWALL_HI:
                            if (S2[i * PART_MAXVAR + PART_ZPOS] > W[j].zmax && S2[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmax;
                                S2[i * PART_MAXVAR + PART_ZVEL] = 0.985*S1[i * PART_MAXVAR + PART_ZVEL];
                                if (S2[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                }
                            }
                            break;
                   
                        default:
                            break;
                    }
                }
            }
        }
        if (isFountain){
            for (i = 0; i < this.partCount; i++){
                S2[i * PART_MAXVAR + PART_AGE] = S1[i * PART_MAXVAR + PART_AGE] - 1; //????????
                if (S2[i * PART_MAXVAR + PART_AGE] < 0){  // Dead
                    // Restart life cycle.
                    this.roundRand();
                    S2[i * PART_MAXVAR + PART_XPOS] = this.randX;
                    S2[i * PART_MAXVAR + PART_YPOS] = this.randY;
                    S2[i * PART_MAXVAR + PART_ZPOS] = this.randZ+1;
                    S2[i * PART_MAXVAR + PART_WPOS] = 1.0;
                    S2[i * PART_MAXVAR + PART_XVEL] = this.INIT_VEL*(0.0 + this.randX);
                    S2[i * PART_MAXVAR + PART_YVEL] = this.INIT_VEL*(0.0 + this.randY);
                    S2[i * PART_MAXVAR + PART_ZVEL] = this.INIT_VEL*(0.5 + this.randZ);
                    S2[i * PART_MAXVAR + PART_AGE] = 30 + 100*Math.random();
                }
            }
        }
        
    }


    /**
     * Rendering all particles
     * @param {Float32Array} S Current State to be drawn
     * @param {*} modelMatrix 
     * @param {*} u_ModelMatrix 
     */
    drawMe(S, modelMatrix, u_ModelMatrix){
        // Draw all particles together
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, S, gl.DYNAMIC_DRAW);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.POINTS, 0, this.partCount);
    }

    
    /**
     * Swap reference of two states. 
     * @param {Float32Array} A Previous State
     * @param {*} B Current State
     */
    stateVecSwap(A, B){
        [B, A] = [A, B];
        return [A, B]
    }


    //--------------Util---------------------
    setArr(arr, val) {  // set array 'arr' to value 'val'
        var i, n = arr.length;
        for (i = 0; i < n; ++i) {
            arr[i] = val;
        }
    }

    roundRand(){  // Find a 3D point randomly and uniformly in a sphere of radius 1.0
        do {
            this.randX = 2.0*Math.random() -1.0;
            this.randY = 2.0*Math.random() -1.0;
            this.randZ = 2.0*Math.random() -1.0;
        }  
        while(this.randX*this.randX + 
              this.randY*this.randY + 
              this.randZ*this.randZ >= 1.0); 
        this.randX *= 0.1;
        this.randY *= 0.1;
        this.randZ *= 0.1; 
    }
}