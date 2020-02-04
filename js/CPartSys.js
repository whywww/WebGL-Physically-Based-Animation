SOLV_EULER = 0;
SOLV_IMPLICIT = 1;
SOLV_MIDPOINT = 2;
SOLV_ME = 3;
SOLV_MAX = 4;
solverType = SOLV_MIDPOINT;

isFountain = 0;
isFixed = true;
isWind = true;

springLen = 0.3;
springStiffness = 2;
springDamp = 0.3;
windVel = [1.0, 0.0, 0.0];
ballRadius = 0.05;  // radius of particle balls


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
    initBouncyBall(partCount, forces, walls){
        this.partCount = partCount;
        this.forcerCount = forces.length;
        this.wallCount = walls.length;

        // initialize s0 for all particles
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S0dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);
        
        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){  // init mass
            this.S0[j + PTYPE_DEAD] = 2;
            this.S0[j + PART_MASS] = 1 + Math.random()*2;
            this.S0[j + PART_XPOS] = 1.5 * (Math.random()-0.5);
            this.S0[j + PART_YPOS] = 1.5 * (Math.random()-0.5);
            this.S0[j + PART_ZPOS] = 0.0;
            this.S0[j + PART_WPOS] = 1.0;
            this.S0[j + PART_XVEL] = 0.0;
            this.S0[j + PART_YVEL] = 0.0;
            this.S0[j + PART_ZVEL] = 0.0;
            this.S0[j + PART_X_FTOT] = 0.0;
            this.S0[j + PART_Y_FTOT] = 0.0;
            this.S0[j + PART_Z_FTOT] = 0.0;
            this.S0[j + PART_MAXAGE] = 30 + 100*Math.random();
            this.S0[j + PART_AGE] = this.S0[j + PART_MAXAGE];
            this.S0[j + PART_MASS_VEL] = 0.0;
            this.S0[j + PART_MASS_FTOT] = 0.0;
            this.S0[j + PART_R] = 1.0;
            this.S0[j + PART_G] = 1.0;
            this.S0[j + PART_B] = 1.0;
        }
 
        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.SMdot.fill(0);
        
        this.solvType = solverType;
        // initialize f0, the list of CForcer for s0
        this.F0 = []; // Do we need single CPart/force object?
        for (var i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));
        }

        // initialize constraints c0 used to adjust s1
        this.C0 = [];
        for (var i = 0; i < this.wallCount; i++){
            this.C0.push(new CWall(walls[i]));
        }
    }


    initSpringSnake(partCount, forces, walls){
        this.partCount = partCount;  // Spring vertex #
        this.forcerCount = forces.length;
        this.wallCount = walls.length;

        // initialize s1 for all particles
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S0dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);

        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){
            this.S0[j + PTYPE_DEAD] = 1;
            this.S0[j + PART_MASS] = 0.01;
            this.S0[j + PART_XPOS] = i*0.1;
            this.S0[j + PART_YPOS] = 0;
            this.S0[j + PART_ZPOS] = 1.5-i*0.17;
            this.S0[j + PART_WPOS] =  1.0;
            this.S0[j + PART_XVEL] =  0.0;
            this.S0[j + PART_YVEL] =  0.0;
            this.S0[j + PART_ZVEL] =  0.0;
            this.S0[j + PART_X_FTOT] =  0.0;
            this.S0[j + PART_Y_FTOT] =  0.0;
            this.S0[j + PART_Z_FTOT] =  0.0;
            this.S0[j + PART_AGE] =  0.0;
            this.S0[j + PART_MASS_VEL] =  0.0;
            this.S0[j + PART_MASS_FTOT] =  0.0;
            this.S0[j + PART_R] =  1.0;
            this.S0[j + PART_G] =  1.0;
            this.S0[j + PART_B] =  1.0;
        }

        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.SMdot.fill(0);

        this.solvType = solverType;
        
        this.F0 = [];
        for (i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));
        }

        this.C0 = [];
        for (i = 0; i < this.wallCount; i++){
            this.C0.push(new CWall(walls[i]));
        }
    }


    initSpringTet(partCount, forces, walls){
        this.partCount = partCount;  // Spring vertex #
        this.forcerCount = forces.length;
        this.wallCount = walls.length;

        // initialize s1 for all particles
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S0dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);

        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){
            this.S0[j + PTYPE_DEAD] = 1;
            this.S0[j + PART_MASS] = 0.01;
            this.S0[j + PART_XPOS] = i*0.1;
            this.S0[j + PART_YPOS] = 0;
            this.S0[j + PART_ZPOS] = 1.5-i*0.17;
            this.S0[j + PART_WPOS] =  1.0;
            this.S0[j + PART_XVEL] =  0.0;
            this.S0[j + PART_YVEL] =  0.0;
            this.S0[j + PART_ZVEL] =  0.0;
            this.S0[j + PART_X_FTOT] =  0.0;
            this.S0[j + PART_Y_FTOT] =  0.0;
            this.S0[j + PART_Z_FTOT] =  0.0;
            this.S0[j + PART_AGE] =  0.0;
            this.S0[j + PART_MASS_VEL] =  0.0;
            this.S0[j + PART_MASS_FTOT] =  0.0;
            this.S0[j + PART_R] =  1.0;
            this.S0[j + PART_G] =  1.0;
            this.S0[j + PART_B] =  1.0;
        }

        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.SMdot.fill(0);

        this.solvType = solverType;
        
        this.F0 = [];
        for (i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));
        }

        this.C0 = [];
        for (i = 0; i < this.wallCount; i++){
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

        // Step through the forcers. Accumulate all forces.
        for (var j = 0; j < this.forcerCount; j++){
            switch(F[j].forceType){
                case -F_GRAV_E:  // disabled gravity. Do nothing.
                    break;
                    
                case F_GRAV_E:  // Earth gravity.
                    for (var i = 0; i < this.partCount; i++){  // For every particle
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

                case F_DRAG:  // viscous drag: force = -velocity*K_drag.
                    for(var i = 0; i < this.partCount; i++){
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

                case F_SPRING:  // Spring force between two particles
                    // F[j].K_springlen = springLen;
                    // F[j].K_spring = springStiffness;
                    // F[j].K_springdamp = springDamp;

                    // var currLen = distance(S.slice(PART_XPOS, PART_ZPOS+1), [S.slice(PART_MAXVAR + PART_XPOS, PART_MAXVAR + PART_ZPOS+1)]);
                    
                    // // Fspring = -K * stretch
                    // var Ftot = -F[j].K_spring * (currLen - F[j].K_springlen)
                    // S[PART_MAXVAR + PART_X_FTOT] += (S[PART_MAXVAR + PART_XPOS] - S[PART_XPOS]) * Ftot / currLen;
                    // S[PART_MAXVAR + PART_Y_FTOT] += (S[PART_MAXVAR + PART_YPOS] - S[PART_YPOS]) * Ftot / currLen;
                    // S[PART_MAXVAR + PART_Z_FTOT] += (S[PART_MAXVAR + PART_ZPOS] - S[PART_ZPOS]) * Ftot / currLen;

                    // // Fdaming = -bv
                    // S[PART_MAXVAR + PART_X_FTOT] += -F[j].K_springdamp * S[PART_MAXVAR + PART_XVEL];
                    // S[PART_MAXVAR + PART_Y_FTOT] += -F[j].K_springdamp * S[PART_MAXVAR + PART_YVEL];
                    // S[PART_MAXVAR + PART_Z_FTOT] += -F[j].K_springdamp * S[PART_MAXVAR + PART_ZVEL];
                    
                    // S[PART_X_FTOT] = -S[PART_MAXVAR + PART_X_FTOT];
                    // S[PART_Y_FTOT] = -S[PART_MAXVAR + PART_Y_FTOT];
                    // S[PART_Z_FTOT] = -S[PART_MAXVAR + PART_Z_FTOT];
                    break;

                case F_SPRING_SNAKE:  // Spring snake
                    F[j].K_springlen = springLen;
                    F[j].K_spring = springStiffness;
                    F[j].K_springdamp = springDamp;

                    for (var i = 0, k = 0; i < this.partCount-1; i++, k+= PART_MAXVAR){
                        var currLen = distance(
                                                S.slice(k+PART_XPOS, k+PART_ZPOS + 1), 
                                                S.slice(k+PART_MAXVAR + PART_XPOS, k+PART_MAXVAR + PART_ZPOS + 1));

                        var Ftot = -F[j].K_spring * (currLen - F[j].K_springlen);

                        S[k + PART_MAXVAR + PART_X_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_XPOS] - S[k + PART_XPOS]) / currLen;
                        S[k + PART_MAXVAR + PART_Y_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_YPOS] - S[k + PART_YPOS]) / currLen;
                        S[k + PART_MAXVAR + PART_Z_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_ZPOS] - S[k + PART_ZPOS]) / currLen;


                        S[k + PART_MAXVAR + PART_X_FTOT] += -F[j].K_springdamp * S[k + PART_MAXVAR + PART_XVEL];
                        S[k + PART_MAXVAR + PART_Y_FTOT] += -F[j].K_springdamp * S[k + PART_MAXVAR + PART_YVEL];
                        S[k + PART_MAXVAR + PART_Z_FTOT] += -F[j].K_springdamp * S[k + PART_MAXVAR + PART_ZVEL];

                        S[k + PART_X_FTOT] += -S[k + PART_MAXVAR + PART_X_FTOT];
                        S[k + PART_Y_FTOT] += -S[k + PART_MAXVAR + PART_Y_FTOT];
                        S[k + PART_Z_FTOT] += -S[k + PART_MAXVAR + PART_Z_FTOT];
                    }
                    if (isFixed){
                        S[PART_X_FTOT] = 0;  // fix to one point
                        S[PART_Y_FTOT] = 0;
                        S[PART_Z_FTOT] = 0;
                    }
                    //  else {
                    //     S[PART_X_FTOT] = -S[PART_MAXVAR + PART_X_FTOT];
                    //     S[PART_Y_FTOT] = -S[PART_MAXVAR + PART_Y_FTOT];
                    //     S[PART_Z_FTOT] = -S[PART_MAXVAR + PART_Z_FTOT];
                    // }
                    
                    // S[(this.partCount-1)*PART_MAXVAR + PART_X_FTOT] = 0.0;  // fix another point
                    // S[(this.partCount-1)*PART_MAXVAR + PART_Y_FTOT] = 0.0;
                    // S[(this.partCount-1)*PART_MAXVAR + PART_Z_FTOT] = 0.0;
                    break;
                
                case F_SPRING_TET:  // Spring tetrahedron
                    F[j].K_springlen = springLen;
                    F[j].K_spring = springStiffness;
                    F[j].K_springdamp = springDamp;
                    // debugger;

                    for (var m = 0; m < this.partCount-1; m++){
                        for (var i = m + 1; i < this.partCount; i++){  // for every other particles
                            var currLen = distance(
                                                S.slice(i*PART_MAXVAR + PART_XPOS, i*PART_MAXVAR + PART_ZPOS + 1), 
                                                S.slice(m*PART_MAXVAR + PART_XPOS, m*PART_MAXVAR + PART_ZPOS + 1));

                            var Ftot = -F[j].K_spring * (currLen - F[j].K_springlen);

                            S[m*PART_MAXVAR + PART_X_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_XPOS] - S[i*PART_MAXVAR + PART_XPOS]) / currLen;
                            S[m*PART_MAXVAR + PART_Y_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_YPOS] - S[i*PART_MAXVAR + PART_YPOS]) / currLen;
                            S[m*PART_MAXVAR + PART_Z_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_ZPOS] - S[i*PART_MAXVAR + PART_ZPOS]) / currLen;

                            S[m*PART_MAXVAR + PART_X_FTOT] += -F[j].K_springdamp * S[m*PART_MAXVAR + PART_XVEL];
                            S[m*PART_MAXVAR + PART_Y_FTOT] += -F[j].K_springdamp * S[m*PART_MAXVAR+ PART_YVEL];
                            S[m*PART_MAXVAR + PART_Z_FTOT] += -F[j].K_springdamp * S[m*PART_MAXVAR + PART_ZVEL];

                            S[i*PART_MAXVAR + PART_X_FTOT] += -S[m*PART_MAXVAR + PART_X_FTOT];
                            S[i*PART_MAXVAR + PART_Y_FTOT] += -S[m*PART_MAXVAR + PART_Y_FTOT];
                            S[i*PART_MAXVAR + PART_Z_FTOT] += -S[m*PART_MAXVAR + PART_Z_FTOT];
                        }
                    }

                    break;
                
                case F_WIND:
                    if (isWind){
                        F[j].D_wind = 0.1;
                        F[j].v_wind = windVel;
    
                        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                            S[k + PART_X_FTOT] += F[j].D_wind * F[j].v_wind[0];
                            S[k + PART_Y_FTOT] += F[j].D_wind * F[j].v_wind[1];
                            S[k + PART_Z_FTOT] += F[j].D_wind * F[j].v_wind[2];
                        }
                    }
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
        for (var i = 0; i < this.partCount; i++){  // For every particle
            
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
        this.solvType = solverType;
        switch (this.solvType){
            case SOLV_EULER:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S0dot[k + PART_XPOS] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S0dot[k + PART_YPOS] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S0dot[k + PART_ZPOS] * timeStep * 0.001;

                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL] * timeStep * 0.001;
                }
                break;
            case SOLV_IMPLICIT:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // IMPLICIT. Compute pos & vel for new state vector
                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL]* timeStep *0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL]* timeStep *0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL]* timeStep *0.001;
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S1[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S1[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S1[k + PART_ZVEL] * timeStep * 0.001;
                }
                break;
            case SOLV_MIDPOINT:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // Clear SM and SMdot
                    this.SM.set(S0);
                    this.SMdot.fill(0);

                    // Use Euler to find midpoint.
                    this.SM[k + PART_XPOS] = S0[k + PART_XPOS] + S0dot[k + PART_XPOS] * timeStep/2 * 0.001;
                    this.SM[k + PART_YPOS] = S0[k + PART_YPOS] + S0dot[k + PART_YPOS] * timeStep/2 * 0.001;
                    this.SM[k + PART_ZPOS] = S0[k + PART_ZPOS] + S0dot[k + PART_ZPOS] * timeStep/2 * 0.001;

                    this.SM[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL] * timeStep/2 * 0.001;
                    this.SM[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL] * timeStep/2 * 0.001;
                    this.SM[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL] * timeStep/2 * 0.001;

                    // Find SMdot
                    this.dotMaker(this.SMdot, this.SM);

                    // use Euler and SMdot
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + this.SMdot[k + PART_XPOS] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + this.SMdot[k + PART_YPOS] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + this.SMdot[k + PART_ZPOS] * timeStep * 0.001;

                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + this.SMdot[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + this.SMdot[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + this.SMdot[k + PART_ZVEL] * timeStep * 0.001;
                }
                break;
            case SOLV_ME:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // implicit
                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL]* timeStep *0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL]* timeStep *0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL]* timeStep *0.001;
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S1[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S1[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S1[k + PART_ZVEL] * timeStep * 0.001;
                    
                    // euler
                    S1[k + PART_XPOS] += S0[k + PART_XPOS] + S0dot[k + PART_XPOS] * timeStep * 0.001;
                    S1[k + PART_YPOS] += S0[k + PART_YPOS] + S0dot[k + PART_YPOS] * timeStep * 0.001;
                    S1[k + PART_ZPOS] += S0[k + PART_ZPOS] + S0dot[k + PART_ZPOS] * timeStep * 0.001;
                    S1[k + PART_XVEL] += S0[k + PART_XVEL] + S0dot[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YVEL] += S0[k + PART_YVEL] + S0dot[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZVEL] += S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL] * timeStep * 0.001;

                    // Average results of implicit & explicit solver
                    S1[k + PART_XPOS] /= 2;
                    S1[k + PART_YPOS] /= 2;
                    S1[k + PART_ZPOS] /= 2;
                    S1[k + PART_XVEL] /= 2;
                    S1[k + PART_YVEL] /= 2;
                    S1[k + PART_ZVEL] /= 2;

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
        for (var j = 0; j < this.wallCount; j++){
            if (W[j].partSetSize == 0){  // limit applied on all particles

                    switch(W[j].wallType){  
                        case WTYPE_GROUND:
                            for (var i = 0; i < this.partCount; i++){
                            if (S2[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                                S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                                S2[i * PART_MAXVAR + PART_ZVEL] = 0.985*S1[i * PART_MAXVAR + PART_ZVEL];
                                if (S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                }
                            }}
                            break;

                        case WTYPE_YWALL_LO:
                            for (var i = 0; i < this.partCount; i++){
                            if (S2[i * PART_MAXVAR + PART_YPOS] < W[j].ymin && S2[i * PART_MAXVAR + PART_YVEL] < 0.0){  // collision
                                S2[i * PART_MAXVAR + PART_YPOS] = W[j].ymin;
                                S2[i * PART_MAXVAR + PART_YVEL] = S1[i * PART_MAXVAR + PART_YVEL]; // Still apply drag??
                                if (S2[i * PART_MAXVAR + PART_YVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                                }
                            }}
                            break;

                        case WTYPE_YWALL_HI:
                            for (var i = 0; i < this.partCount; i++){
                            if (S2[i * PART_MAXVAR + PART_YPOS] > W[j].ymax && S2[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                S2[i * PART_MAXVAR + PART_YPOS] = W[j].ymax;
                                S2[i * PART_MAXVAR + PART_YVEL] = S1[i * PART_MAXVAR + PART_YVEL];
                                if (S2[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                                }
                            }}
                            break;

                        case WTYPE_XWALL_LO:
                            for (var i = 0; i < this.partCount; i++){
                            if (S2[i * PART_MAXVAR + PART_XPOS] < W[j].xmin && S2[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                S2[i * PART_MAXVAR + PART_XPOS] = W[j].xmin;
                                S2[i * PART_MAXVAR + PART_XVEL] = S1[i * PART_MAXVAR + PART_XVEL];
                                if (S2[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                    S2[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                                }
                            }}
                            break;

                        case WTYPE_XWALL_HI:
                            for (var i = 0; i < this.partCount; i++){
                            if (S2[i * PART_MAXVAR + PART_XPOS] > W[j].xmax && S2[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                S2[i * PART_MAXVAR + PART_XPOS] = W[j].xmax;
                                S2[i * PART_MAXVAR + PART_XVEL] = S1[i * PART_MAXVAR + PART_XVEL];
                                if (S2[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                                } else {
                                    S2[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                                }
                            }}
                            break;

                        case WTYPE_ZWALL_LO:
                            for (var i = 0; i < this.partCount; i++){
                                if (S2[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                                    S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                                    S2[i * PART_MAXVAR + PART_ZVEL] = S1[i * PART_MAXVAR + PART_ZVEL];
                                    if (S2[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                        S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                    } else {
                                        S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                    }
                                }
                            }
                            break;

                        case WTYPE_ZWALL_HI:
                            for (var i = 0; i < this.partCount; i++){
                                if (S2[i * PART_MAXVAR + PART_ZPOS] > W[j].zmax && S2[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                    S2[i * PART_MAXVAR + PART_ZPOS] = W[j].zmax;
                                    S2[i * PART_MAXVAR + PART_ZVEL] = S1[i * PART_MAXVAR + PART_ZVEL];
                                    if (S2[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                        S2[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                                    } else {
                                        S2[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                                    }
                                }
                            }
                            break;

                        case WTYPE_AGE:
                            if (isFountain){
                                for (var i = 0; i < this.partCount; i++){
                                    S2[i * PART_MAXVAR + PART_AGE] = S1[i * PART_MAXVAR + PART_AGE] - 1;
                                    // S2[i * PART_MAXVAR + PART_MASS] = S1[i * PART_MAXVAR + PART_MASS] - 0.01;

                                    if (S2[i * PART_MAXVAR + PART_AGE] > 0.8 * S2[i * PART_MAXVAR + PART_MAXAGE]){
                                        S2[i * PART_MAXVAR + PART_B] = S2[i * PART_MAXVAR + PART_AGE]/0.2/S2[i * PART_MAXVAR + PART_MAXAGE] - 4;
                                    }
                                    else if (S2[i * PART_MAXVAR + PART_AGE] > 0.6 * S2[i * PART_MAXVAR + PART_MAXAGE]){
                                        S2[i * PART_MAXVAR + PART_B] = 0;
                                        S2[i * PART_MAXVAR + PART_G] = S2[i * PART_MAXVAR + PART_AGE]/0.2/S2[i * PART_MAXVAR + PART_MAXAGE] - 3;
                                    }
                                    else {
                                        S2[i * PART_MAXVAR + PART_G] = 0;
                                        S2[i * PART_MAXVAR + PART_R] = S2[i * PART_MAXVAR + PART_AGE]/0.6/S2[i * PART_MAXVAR + PART_MAXAGE];
                                    }
                                    
                                    if (S2[i * PART_MAXVAR + PART_AGE] < 0 ){  // Dead
                                        // Restart life cycle.
                                        this.roundRand();
                                        S2[i * PART_MAXVAR + PART_MASS] = 1 + Math.random()*2;
                                        S2[i * PART_MAXVAR + PART_XPOS] = this.randX;
                                        S2[i * PART_MAXVAR + PART_YPOS] = this.randY;
                                        S2[i * PART_MAXVAR + PART_ZPOS] = this.randZ+1;
                                        S2[i * PART_MAXVAR + PART_WPOS] = 1.0;
                                        S2[i * PART_MAXVAR + PART_XVEL] = this.INIT_VEL*(0.0 + this.randX);
                                        S2[i * PART_MAXVAR + PART_YVEL] = this.INIT_VEL*(0.0 + this.randY);
                                        S2[i * PART_MAXVAR + PART_ZVEL] = this.INIT_VEL*(0.5 + this.randZ);
                                        S2[i * PART_MAXVAR + PART_AGE] = S2[i * PART_MAXVAR + PART_MAXAGE];
                                        S2[i * PART_MAXVAR + PART_R] = 1.0;
                                        S2[i * PART_MAXVAR + PART_G] = 1.0;
                                        S2[i * PART_MAXVAR + PART_B] = 1.0;
                                    }
                                    
                                }
                            }
                            break;

                        case WTYPE_PBALL:
                            for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                                var dis = distance(pBallCenter, S2.slice(k + PART_XPOS, k + PART_ZPOS + 1));
                                if (dis < pBallRadius + ballRadius){
                                    // debugger;
                                    S2[k + PART_XPOS] = S1[k + PART_XPOS] - xMdragTot;
                                    S2[k + PART_ZPOS] = S1[k + PART_ZPOS] + yMdragTot;
                                    
                                    S2[k + PART_XVEL] = S1[k + PART_XVEL];
                                    S2[k + PART_YVEL] = S1[k + PART_YVEL];
                                    S2[k + PART_ZVEL] = S1[k + PART_ZVEL];

                                    S2[k + PART_XVEL] *= -W[j].Kbouncy;
                                    S2[k + PART_YVEL] *= -W[j].Kbouncy;
                                    S2[k + PART_ZVEL] *= -W[j].Kbouncy;
                                }
                            }
                            break;
                   
                        default:
                            break;
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
    roundRand(){  // Find a 3D point randomly and uniformly in a sphere of radius 0.1
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

function distance(p1, p2){  // Calculate distance between two 3d points
        var x = Math.abs(p1[0] - p2[0]);
        var y = Math.abs(p1[1] - p2[1]);
        var z = Math.abs(p1[2] - p2[2]);

        return Math.sqrt(x*x + y*y + z*z);
    }