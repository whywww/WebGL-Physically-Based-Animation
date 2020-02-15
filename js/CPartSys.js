SOLV_EULER = 0;
SOLV_IM_EULER = 1;
SOLV_EX_MIDPOINT = 2;
SOLV_ITER_IM_EULER = 3;
SOLV_ITER_IM_MIDPOINT = 4;
SOLV_ADAMS_BASHFORTH = 5;
SOLV_VEL_VERLET = 6;
SOLV_ME = 7;
SOLV_MAX = 8;
solverType = SOLV_EX_MIDPOINT;

isFountain = false;
isFixed = true;

springLen = 0.3;
springStiffness = 4;
springDamp = 0.1;
windVel = [1.0, 0.0, 0.0];
T_center = [0, 0, 0];
sprMass = 0.5;

cnt = 0;

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
        this.S1dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);
        
        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){  // init mass
            this.S0[j + PART_MASS] = 1 ;
            this.S0[j + PART_XPOS] = 1.5 * (Math.random()-0.5);
            this.S0[j + PART_YPOS] = 0;
            this.S0[j + PART_ZPOS] = 0;
            this.S0[j + PART_WPOS] = 1.0;
            this.S0[j + PART_XVEL] = Math.random();
            this.S0[j + PART_YVEL] = 0;
            this.S0[j + PART_ZVEL] = Math.random();
            this.S0[j + PART_X_FTOT] = 0.0;
            this.S0[j + PART_Y_FTOT] = 0.0;
            this.S0[j + PART_Z_FTOT] = 0.0;
            this.S0[j + PART_SIZE] = 8.0;
            this.S0[j + PART_MAXAGE] = 30 + 100*Math.random();
            this.S0[j + PART_AGE] = this.S0[j + PART_MAXAGE];
            this.S0[j + PART_R] = 0.3;
            this.S0[j + PART_G] = 0.3;
            this.S0[j + PART_B] = 0.3;
            this.S0[j + PART_A] = 0.9;
        }
 
        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.S1dot.fill(0);
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
        this.S1dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);

        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){
            this.S0[j + PART_MASS] = sprMass;
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
            this.S0[j + PART_SIZE] = 20.0;
            this.S0[j + PART_AGE] =  0.0;
            this.S0[j + PART_R] =  1.0;
            this.S0[j + PART_G] =  1.0;
            this.S0[j + PART_B] =  1.0;
            this.S0[j + PART_A] =  1.0;
        }

        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.S1dot.fill(0);
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
        this.S1dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);

        // Do all initializations here
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){
            this.S0[j + PART_MASS] = sprMass;
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
            this.S0[j + PART_SIZE] = 20.0;
            this.S0[j + PART_AGE] =  0.0;
            this.S0[j + PART_R] =  1.0;
            this.S0[j + PART_G] =  1.0;
            this.S0[j + PART_B] =  1.0;
            this.S0[j + PART_A] =  1.0;
        }

        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.S1dot.fill(0);
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
        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
            S[k + PART_X_FTOT] = 0.0;
            S[k + PART_Y_FTOT] = 0.0;
            S[k + PART_Z_FTOT] = 0.0;            
        }
        
        // Step through the forcers. Accumulate all forces.
        for (var j = 0; j < this.forcerCount; j++){
            switch(F[j].forceType){
                case -F_GRAV_E:  // disabled gravity. Do nothing.
                    break;
                    
                case F_GRAV_E:  // Earth gravity.
                    for (var i = 0; i < this.partCount; i++){  // For every particle
                        // magnitude of gravity's force. F=ma
                        var mag = S[i * PART_MAXVAR + PART_MASS] * F[j].gravConst;  
                        // make a vector: scale our unit vector in the 'down' direction:
                        S[i * PART_MAXVAR + PART_X_FTOT] += mag * F[j].downDir[0];
                        S[i * PART_MAXVAR + PART_Y_FTOT] += mag * F[j].downDir[1];
                        S[i * PART_MAXVAR + PART_Z_FTOT] += mag * F[j].downDir[2];
                    }
                    break;

                case F_DRAG:  // viscous drag: force = -velocity*K_drag.
                    for(var i = 0; i < this.partCount; i++){
                        // add to force-accumulator
                        S[i * PART_MAXVAR + PART_X_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_XVEL];
                        S[i * PART_MAXVAR + PART_Y_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_YVEL];
                        S[i * PART_MAXVAR + PART_Z_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_ZVEL];
                    }
                    break;

                case F_SPRING_SNAKE:  // Spring snake
                    F[j].K_springlen = springLen;
                    F[j].K_spring = springStiffness;
                    F[j].K_springdamp = springDamp;

                    for (var i = 0, k = 0; i < this.partCount-1; i++, k+= PART_MAXVAR){
                        var currLen = distance3D(
                                                S.slice(k+PART_XPOS, k+PART_ZPOS + 1), 
                                                S.slice(k+PART_MAXVAR + PART_XPOS, k+PART_MAXVAR + PART_ZPOS + 1));

                        var Ftot = -F[j].K_spring * (currLen - F[j].K_springlen);

                        S[k + PART_MAXVAR + PART_X_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_XPOS] - S[k + PART_XPOS]) / currLen;
                        S[k + PART_MAXVAR + PART_Y_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_YPOS] - S[k + PART_YPOS]) / currLen;
                        S[k + PART_MAXVAR + PART_Z_FTOT] += Ftot * (S[k + PART_MAXVAR + PART_ZPOS] - S[k + PART_ZPOS]) / currLen;

                        S[k + PART_MAXVAR + PART_X_FTOT] += -F[j].K_springdamp * (S[k + PART_MAXVAR + PART_XVEL] - S[k + PART_XVEL])/2;
                        S[k + PART_MAXVAR + PART_Y_FTOT] += -F[j].K_springdamp * (S[k + PART_MAXVAR + PART_YVEL] - S[k + PART_YVEL])/2;
                        S[k + PART_MAXVAR + PART_Z_FTOT] += -F[j].K_springdamp * (S[k + PART_MAXVAR + PART_ZVEL] - S[k + PART_ZVEL])/2;

                        S[k + PART_X_FTOT] += -S[k + PART_MAXVAR + PART_X_FTOT];
                        S[k + PART_Y_FTOT] += -S[k + PART_MAXVAR + PART_Y_FTOT];
                        S[k + PART_Z_FTOT] += -S[k + PART_MAXVAR + PART_Z_FTOT];
                    }
                    
                    // fix to one point
                    if (isFixed){ 
                        S[PART_X_FTOT] = 0; 
                        S[PART_Y_FTOT] = 0;
                        S[PART_Z_FTOT] = 0;
                    }
                    break;
                
                case F_SPRING_TET:  // Spring tetrahedron
                    F[j].K_springlen = springLen;
                    F[j].K_spring = springStiffness;
                    F[j].K_springdamp = springDamp;

                    for (var m = 0; m < this.partCount-1; m++){
                        for (var i = m + 1; i < this.partCount; i++){  // for every other particles
                            var currLen = distance3D(
                                                S.slice(i*PART_MAXVAR + PART_XPOS, i*PART_MAXVAR + PART_ZPOS + 1), 
                                                S.slice(m*PART_MAXVAR + PART_XPOS, m*PART_MAXVAR + PART_ZPOS + 1));

                            var Ftot = -F[j].K_spring * (currLen - F[j].K_springlen);

                            S[m*PART_MAXVAR + PART_X_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_XPOS] - S[i*PART_MAXVAR + PART_XPOS]) / currLen;
                            S[m*PART_MAXVAR + PART_Y_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_YPOS] - S[i*PART_MAXVAR + PART_YPOS]) / currLen;
                            S[m*PART_MAXVAR + PART_Z_FTOT] += Ftot * (S[m*PART_MAXVAR + PART_ZPOS] - S[i*PART_MAXVAR + PART_ZPOS]) / currLen;

                            S[m*PART_MAXVAR + PART_X_FTOT] += -F[j].K_springdamp * (S[m*PART_MAXVAR + PART_XVEL] - S[i*PART_MAXVAR + PART_XVEL])/2;
                            S[m*PART_MAXVAR + PART_Y_FTOT] += -F[j].K_springdamp * (S[m*PART_MAXVAR+ PART_YVEL] - S[i*PART_MAXVAR + PART_YVEL])/2;
                            S[m*PART_MAXVAR + PART_Z_FTOT] += -F[j].K_springdamp * (S[m*PART_MAXVAR + PART_ZVEL] - S[i*PART_MAXVAR + PART_ZVEL])/2;

                            S[i*PART_MAXVAR + PART_X_FTOT] += -S[m*PART_MAXVAR + PART_X_FTOT];
                            S[i*PART_MAXVAR + PART_Y_FTOT] += -S[m*PART_MAXVAR + PART_Y_FTOT];
                            S[i*PART_MAXVAR + PART_Z_FTOT] += -S[m*PART_MAXVAR + PART_Z_FTOT];
                        }
                    }
                    break;
                
                case F_WIND:
                        F[j].D_wind = 0.1;
                        F[j].v_wind = windVel;
    
                        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                            S[k + PART_X_FTOT] += F[j].D_wind * F[j].v_wind[0];
                            S[k + PART_Y_FTOT] += F[j].D_wind * F[j].v_wind[1];
                            S[k + PART_Z_FTOT] += F[j].D_wind * F[j].v_wind[2];
                        }
                    break;
                
                case F_TORNADO:
                        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                            var dis = distance2D(T_center.slice(0,2), 
                                                S.slice(k + PART_XPOS, k + PART_ZPOS));
                            // var inStrength = Math.pow(10, 1/dis/dis); 
                            var inStrength = 1/(dis*dis/dis);
    
                            // upward force F += constant strength
                            S[k + PART_Z_FTOT] += inStrength;
                            
                            // inward force, F += strength * normalized direction
                            S[k + PART_X_FTOT] += (T_center[0] - S[k + PART_XPOS])/dis;
                            S[k + PART_Y_FTOT] += (T_center[1] - S[k + PART_YPOS])/dis;
                            if (S[k + PART_X_FTOT] > 0){S[k + PART_X_FTOT] -= Math.pow(S[k + PART_ZPOS] - T_center[2], 2);}
                            else S[k + PART_X_FTOT] += Math.pow(S[k + PART_ZPOS] - T_center[2], 2);

                            if (S[k + PART_Y_FTOT] > 0){S[k + PART_Y_FTOT] -= Math.pow(S[k + PART_ZPOS] - T_center[2], 2);}
                            else S[k + PART_Y_FTOT] += Math.pow(S[k + PART_ZPOS] - T_center[2], 2);

                            // spiral force. spining in direction (-y, x), tangent of circle, F += strength * normalized direction
                            S[k + PART_X_FTOT] += (-S[k + PART_YPOS] + T_center[1])/dis;
                            S[k + PART_Y_FTOT] += (S[k + PART_XPOS] - T_center[0])/dis;
                        }
                    break;

                case F_FLOCK:
                    var Ka = 0.4;  // seperation coefficient
                    var Kv = 5;  // alignment coefficient
                    var Kc = 5;  // cohension coefficient
                    var Ktheta = 1; // weighting influence by visual field
                    var Kd = 0;  // weighting factor by distance
                    var Fmax = [1, 1, 1];
                    debugger;

                    for (var i = 0; i < this.partCount; i++){        
                        
                        var dis = distance3D(pBallCenter, S.slice(i*PART_MAXVAR + PART_XPOS, i*PART_MAXVAR + PART_ZPOS + 1));
                        if (dis > pBallRadius + ballRadius){
                            S[i*PART_MAXVAR + PART_X_FTOT] += 0.5*(pBallCenter[0] - S[i*PART_MAXVAR + PART_XPOS]);
                            S[i*PART_MAXVAR + PART_Y_FTOT] += 0.5*(pBallCenter[1] - S[i*PART_MAXVAR + PART_YPOS]);
                            S[i*PART_MAXVAR + PART_Z_FTOT] += 0.5*(pBallCenter[2] - S[i*PART_MAXVAR + PART_ZPOS]);
                        }

                        for (var m = 0; m < this.partCount; m++){  // for every other particle
                            
                            if (i == m){
                                continue;
                            }

                            var Fr = Fmax;  // residual force total

                            // calculate the distance of i and m.
                            var realDis = distance3D(S.slice(i*PART_MAXVAR+PART_XPOS, i*PART_MAXVAR+PART_ZPOS+1),
                                                    S.slice(m*PART_MAXVAR+PART_XPOS, m*PART_MAXVAR+PART_ZPOS+1));

                            // if m is in i's neighbour outer radius
                            if (realDis < F[j].neighborOuterR){
                                Kd = (F[j].neighborOuterR - realDis) / (F[j].neighborOuterR - F[j].neighborInnerR);
                                if (realDis < F[j].neighborInnerR){
                                    Kd = 1;
                                }

                                // theta1: Frontal binocular, theta2: Peripheral monocular
                                var theta1 = 30/180*Math.PI; var theta2 = 340/180*Math.PI;
                                // theta: angle formed from i to m's direction
                                var normM = distance3D([0,0,0], S.slice(m*PART_MAXVAR+PART_XVEL, m*PART_MAXVAR+PART_ZVEL+1));
                                var normI = distance3D([0,0,0], S.slice(i*PART_MAXVAR+PART_XVEL, i*PART_MAXVAR+PART_ZVEL+1));
                                var cosTheta = dot(S.slice(i*PART_MAXVAR+PART_XVEL, i*PART_MAXVAR+PART_ZVEL+1),
                                                    S.slice(m*PART_MAXVAR+PART_XVEL, m*PART_MAXVAR+PART_ZVEL+1))/normM/normI;
                                var theta = Math.acos(cosTheta);
    
                                // adjust Ktheta according to orientation
                                if (theta <= theta2/2 && theta >= theta1/2){
                                    Ktheta = (theta2 / 2 - theta)/(theta2/2-theta1/2);
                                } else if (theta > theta2/2){
                                    Ktheta = 0;
                                }
                                
                                // Seperation
                                var Fax = Math.min(Ka, Fr[0]) * (S[i*PART_MAXVAR+PART_XPOS] - S[m*PART_MAXVAR+PART_XPOS]) / realDis;
                                var Fay = Math.min(Ka, Fr[1]) * (S[i*PART_MAXVAR+PART_YPOS] - S[m*PART_MAXVAR+PART_YPOS]) / realDis;
                                var Faz = Math.min(Ka, Fr[2]) * (S[i*PART_MAXVAR+PART_ZPOS] - S[m*PART_MAXVAR+PART_ZPOS]) / realDis;
                                Fr = [Fr[0]-Math.abs(Fax), Fr[1]-Math.abs(Fay), Fr[2]-Math.abs(Faz)];
                                
                                // Alignment
                                var Fvx = Kv * (S[m*PART_MAXVAR+PART_XVEL] - S[i*PART_MAXVAR+PART_XVEL]);
                                var Fvy = Kv * (S[m*PART_MAXVAR+PART_YVEL] - S[i*PART_MAXVAR+PART_YVEL]);
                                var Fvz = Kv * (S[m*PART_MAXVAR+PART_ZVEL] - S[i*PART_MAXVAR+PART_ZVEL]);
                                var tot = Math.sqrt(Fvx*Fvx+Fvy*Fvy+Fvz*Fvz);
                                Fvx = Math.min(Fr[0], tot) * Fvx/tot;
                                Fvy = Math.min(Fr[1], tot) * Fvy/tot;
                                Fvz = Math.min(Fr[2], tot) * Fvz/tot;
                                Fr = [Fr[0]-Math.abs(Fvx), Fr[1]-Math.abs(Fvy), Fr[2]-Math.abs(Fvz)];
                                
                                // Cohension
                                var Fcx = Kc * (S[m*PART_MAXVAR+PART_XPOS] - S[i*PART_MAXVAR+PART_XPOS]);
                                var Fcy = Kc * (S[m*PART_MAXVAR+PART_YPOS] - S[i*PART_MAXVAR+PART_YPOS]);
                                var Fcz = Kc * (S[m*PART_MAXVAR+PART_ZPOS] - S[i*PART_MAXVAR+PART_ZPOS]);
                                var tot = Math.sqrt(Fcx*Fcx+Fcy*Fcy+Fcz*Fcz);
                                Fcx = Math.min(Fr[0], tot) * Fcx/tot;
                                Fcy = Math.min(Fr[1], tot) * Fcy/tot;
                                Fcz = Math.min(Fr[2], tot) * Fcz/tot;

                                S[i*PART_MAXVAR + PART_X_FTOT] += Ktheta * Kd * (Fax + Fvx + Fcx);
                                S[i*PART_MAXVAR + PART_Y_FTOT] += Ktheta * Kd * (Fay + Fvy + Fcy);
                                S[i*PART_MAXVAR + PART_Z_FTOT] += Ktheta * Kd * (Faz + Fvz + Fcz);
                                
                            }
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
            case SOLV_IM_EULER:
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
            case SOLV_EX_MIDPOINT:
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
            case SOLV_ITER_IM_EULER:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // S2_0
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S0dot[k + PART_XPOS] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S0dot[k + PART_YPOS] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S0dot[k + PART_ZPOS] * timeStep * 0.001;

                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL] * timeStep * 0.001;

                    // S3_0
                    this.dotMaker(this.S1dot, S1);

                    // S_error
                    var error = []
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_XPOS] - S0dot[k + PART_XPOS]));
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_YPOS] - S0dot[k + PART_YPOS]));
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_ZPOS] - S0dot[k + PART_ZPOS]));
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_XVEL] - S0dot[k + PART_XVEL]));
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_YVEL] - S0dot[k + PART_YVEL]));
                    error.push(timeStep * 0.001 * (this.S1dot[k + PART_ZVEL] - S0dot[k + PART_ZVEL]));

                    S1[k + PART_XPOS] = S1[k + PART_XPOS] + 0.5 * error[0];
                    S1[k + PART_YPOS] = S1[k + PART_YPOS] + 0.5 * error[1];
                    S1[k + PART_ZPOS] = S1[k + PART_ZPOS] + 0.5 * error[2];
                    S1[k + PART_XVEL] = S1[k + PART_XVEL] + 0.5 * error[3];
                    S1[k + PART_YVEL] = S1[k + PART_YVEL] + 0.5 * error[4];
                    S1[k + PART_ZVEL] = S1[k + PART_ZVEL] + 0.5 * error[5];
                }
                break;
            case SOLV_ITER_IM_MIDPOINT:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // Clear SM and SMdot
                    this.SM.set(S0);
                    this.SMdot.fill(0);

                    // Use Euler to find next state
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S0dot[k + PART_XPOS] * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S0dot[k + PART_YPOS] * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S0dot[k + PART_ZPOS] * timeStep * 0.001;

                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + S0dot[k + PART_XVEL] * timeStep * 0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + S0dot[k + PART_YVEL] * timeStep * 0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + S0dot[k + PART_ZVEL] * timeStep * 0.001;

                    this.dotMaker(this.S1dot, S1);

                    // find backward midpoint
                    this.SM[k + PART_XPOS] = S1[k + PART_XPOS] + this.S1dot[k + PART_XPOS] * timeStep/2 * 0.001;
                    this.SM[k + PART_YPOS] = S1[k + PART_YPOS] + this.S1dot[k + PART_YPOS] * timeStep/2 * 0.001;
                    this.SM[k + PART_ZPOS] = S1[k + PART_ZPOS] + this.S1dot[k + PART_ZPOS] * timeStep/2 * 0.001;

                    this.SM[k + PART_XVEL] = S1[k + PART_XVEL] + this.S1dot[k + PART_XVEL] * timeStep/2 * 0.001;
                    this.SM[k + PART_YVEL] = S1[k + PART_YVEL] + this.S1dot[k + PART_YVEL] * timeStep/2 * 0.001;
                    this.SM[k + PART_ZVEL] = S1[k + PART_ZVEL] + this.S1dot[k + PART_ZVEL] * timeStep/2 * 0.001;

                    // Find SMdot
                    this.dotMaker(this.SMdot, this.SM);

                    // Error
                    var error = []
                    error.push(S1[k + PART_XPOS] - this.SMdot[k + PART_XPOS] * timeStep * 0.001 - S0[k + PART_XPOS]);
                    error.push(S1[k + PART_YPOS] - this.SMdot[k + PART_YPOS] * timeStep * 0.001 - S0[k + PART_YPOS]);
                    error.push(S1[k + PART_ZPOS] - this.SMdot[k + PART_ZPOS] * timeStep * 0.001 - S0[k + PART_ZPOS]);
                    error.push(S1[k + PART_XVEL] - this.SMdot[k + PART_XVEL] * timeStep * 0.001 - S0[k + PART_XVEL]);
                    error.push(S1[k + PART_YVEL] - this.SMdot[k + PART_YVEL] * timeStep * 0.001 - S0[k + PART_YVEL]);
                    error.push(S1[k + PART_ZVEL] - this.SMdot[k + PART_ZVEL] * timeStep * 0.001 - S0[k + PART_ZVEL]);

                    S1[k + PART_XPOS] = S1[k + PART_XPOS] - 0.5 * error[0];
                    S1[k + PART_YPOS] = S1[k + PART_YPOS] - 0.5 * error[1];
                    S1[k + PART_ZPOS] = S1[k + PART_ZPOS] - 0.5 * error[2];
                    S1[k + PART_XVEL] = S1[k + PART_XVEL] - 0.5 * error[3];
                    S1[k + PART_YVEL] = S1[k + PART_YVEL] - 0.5 * error[4];
                    S1[k + PART_ZVEL] = S1[k + PART_ZVEL] - 0.5 * error[5];
                }
                break;
            case SOLV_ADAMS_BASHFORTH:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // previous sdot
                    this.dotMaker(this.S1dot, S1);

                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + (S0dot[k + PART_XPOS]*3/2 - this.S1dot[k + PART_XPOS]/2) * timeStep * 0.001;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + (S0dot[k + PART_YPOS]*3/2 - this.S1dot[k + PART_YPOS]/2) * timeStep * 0.001;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + (S0dot[k + PART_ZPOS]*3/2 - this.S1dot[k + PART_ZPOS]/2) * timeStep * 0.001;
                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + (S0dot[k + PART_XVEL]*3/2 - this.S1dot[k + PART_XVEL]/2) * timeStep * 0.001;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + (S0dot[k + PART_YVEL]*3/2 - this.S1dot[k + PART_YVEL]/2) * timeStep * 0.001;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + (S0dot[k + PART_ZVEL]*3/2 - this.S1dot[k + PART_ZVEL]/2) * timeStep * 0.001;
                }
                break;

            case SOLV_VEL_VERLET:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    // compute next position quadratically
                    S1[k + PART_XPOS] = S0[k + PART_XPOS] + S0[k + PART_XVEL] * timeStep * 0.001 + S0dot[k + PART_XVEL] * Math.pow(timeStep * 0.001, 2)*0.5;
                    S1[k + PART_YPOS] = S0[k + PART_YPOS] + S0[k + PART_YVEL] * timeStep * 0.001 + S0dot[k + PART_YVEL] * Math.pow(timeStep * 0.001, 2)*0.5;
                    S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + S0[k + PART_ZVEL] * timeStep * 0.001 + S0dot[k + PART_ZVEL] * Math.pow(timeStep * 0.001, 2)*0.5;

                    // find next acc from next pos
                    this.applyAllForces(S1, this.F0);
                    this.dotMaker(this.S1dot, S1);
                    S1[k + PART_XVEL] = S0[k + PART_XVEL] + (this.S1dot[k + PART_XVEL] + S0dot[k + PART_XVEL]) * timeStep * 0.001 * 0.5;
                    S1[k + PART_YVEL] = S0[k + PART_YVEL] + (this.S1dot[k + PART_YVEL] + S0dot[k + PART_YVEL]) * timeStep * 0.001 * 0.5;
                    S1[k + PART_ZVEL] = S0[k + PART_ZVEL] + (this.S1dot[k + PART_ZVEL] + S0dot[k + PART_ZVEL]) * timeStep * 0.001 * 0.5;
                }
                break;
        }
    }


    /**
     * Adjust S1 & S2 to satisfy rules of collisions
     * @param {Float32Array} S1 Next state
     * @param {Float32Array} S0 Curr state
     * @param {Array} W List of Constraints
     */
    doConstraints(S1, S0, W){
        var hasFire = typeof(W.find(w => w.wallType == WTYPE_FIRE));
        
        // step through constraints
        for (var j = 0; j < this.wallCount; j++){
            
            if (hasFire != "undefined"){  // has fire constraint
                W[j].Kbouncy = 0;
            } else {
                W[j].Kbouncy = Kbouncy;
            }

            switch(W[j].wallType){
                case WTYPE_GROUND:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                            S1[i * PART_MAXVAR + PART_ZVEL] = 0.985*S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_YWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_YPOS] < W[j].ymin && S1[i * PART_MAXVAR + PART_YVEL] < 0.0){  // collision
                            S1[i * PART_MAXVAR + PART_YPOS] = W[j].ymin;
                            S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL]; // Still apply drag??
                            if (S1[i * PART_MAXVAR + PART_YVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_YWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_YPOS] > W[j].ymax && S1[i * PART_MAXVAR + PART_YVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_YPOS] = W[j].ymax;
                            S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL];
                            if (S1[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_XWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_XPOS] < W[j].xmin && S1[i * PART_MAXVAR + PART_XVEL] < 0.0){
                            S1[i * PART_MAXVAR + PART_XPOS] = W[j].xmin;
                            S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL];
                            if (S1[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_XWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_XPOS] > W[j].xmax && S1[i * PART_MAXVAR + PART_XVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_XPOS] = W[j].xmax;
                            S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL];
                            if (S1[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_ZWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                            S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_ZWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] > W[j].zmax && S1[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmax;
                            S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_PBALL:
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        var dis = distance3D(pBallCenter, S1.slice(k + PART_XPOS, k + PART_ZPOS + 1));
                        if (dis < pBallRadius + ballRadius){
                            S1[k + PART_XPOS] = S0[k + PART_XPOS] - 2*xMdragTot;
                            S1[k + PART_ZPOS] = S0[k + PART_ZPOS] + 2*yMdragTot;
                            
                            S1[k + PART_XVEL] = S0[k + PART_XVEL];
                            S1[k + PART_YVEL] = S0[k + PART_YVEL];
                            S1[k + PART_ZVEL] = S0[k + PART_ZVEL];

                            S1[k + PART_XVEL] *= -W[j].Kbouncy;
                            S1[k + PART_YVEL] *= -W[j].Kbouncy;
                            S1[k + PART_ZVEL] *= -W[j].Kbouncy;
                        }
                    }
                    break;

                case WTYPE_FIRE:
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        S1[k + PART_AGE] = S0[k+ PART_AGE] - 1;
                        S1[k + PART_MASS] = S0[k + PART_MASS] - 0.05;
                        
                        if (S1[k + PART_MASS] < 0.05){
                            S1[k + PART_MASS] = 0.05;
                        }

                        var lifeLeftPercent = S1[k + PART_AGE]/S1[k + PART_MAXAGE];
                        
                        S1[k + PART_SIZE] = 3 + lifeLeftPercent*7;

                        if (lifeLeftPercent > 0.95){ // white, a little blue
                            S1[k + PART_R] = 1.0;
                            S1[k + PART_G] = 1.0;
                            S1[k + PART_B] = 1.0;
                        }
                        else if (lifeLeftPercent > 0.7){  //yellow to orange
                            S1[k + PART_B] = 0;
                            S1[k + PART_G] = lifeLeftPercent/0.2 - 3.5;
                        }
                        else {  // red to black
                            S1[k + PART_G] = 0;
                            S1[k + PART_R] = lifeLeftPercent/0.7;
                        }
                         
                        if (S1[k + PART_AGE] < 0 ){  // Dead
                            // Restart life cycle.
                            this.roundRand();
                            S1[k + PART_MASS] = 1 + Math.random()*2;
                            S1[k + PART_XPOS] = this.randX;
                            S1[k + PART_YPOS] = this.randY;
                            S1[k + PART_ZPOS] = this.randZ;
                            S1[k + PART_WPOS] = 1.0;
                            S1[k + PART_XVEL] = this.INIT_VEL*(this.randX);
                            S1[k + PART_YVEL] = this.INIT_VEL*(this.randY);
                            S1[k + PART_ZVEL] = this.INIT_VEL*(0.45 + this.randZ);
                            S1[k + PART_SIZE] = 10.0;
                            S1[k + PART_MAXAGE] = 10 + 20*Math.random();
                            S1[k + PART_AGE] = S1[k + PART_MAXAGE];  // Reconstruction here
                            S1[k + PART_R] = 1.0;
                            S1[k + PART_G] = 1.0;
                            S1[k + PART_B] = 1.0;
                            S1[k + PART_A] = 0.7;
                        }  
                    }
                    break;

                case WTYPE_TORNADO:
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        // --Age
                        S1[k + PART_AGE] = S0[k + PART_AGE] - 1;
                        
                        if (S1[k + PART_AGE] < 0 ){  // Dead
                            // Restart life cycle.
                            S1[k + PART_MASS] = 0.1;
                            S1[k + PART_XPOS] = 3 * (Math.random()-0.5);
                            S1[k + PART_YPOS] = 3 * (Math.random()-0.5);
                            S1[k + PART_ZPOS] = 0;
                            S1[k + PART_WPOS] = 1.0;
                            S1[k + PART_XVEL] = 0;
                            S1[k + PART_YVEL] = 0;
                            S1[k + PART_ZVEL] = 0;
                            S1[k + PART_X_FTOT] = 0.0;
                            S1[k + PART_Y_FTOT] = 0.0;
                            S1[k + PART_Z_FTOT] = 0.0;  
                            S1[k + PART_AGE] = 30 + 100*Math.random();
                            S1[k + PART_R] = 0.3;
                            S1[k + PART_G] = 0.3;
                            S1[k + PART_B] = 0.3;
                            S1[k + PART_A] = 0.9;
                        }
                    }
                    break;
            
                default:
                    break;
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
              this.randZ*this.randZ >= 1.0
              && this.randZ < 0.0); 
        this.randX *= 0.1;
        this.randY *= 0.1;
        this.randZ *= 0.1; 
    }

    removeAddForce(fNumbers){
        for (var i = 0; i < fNumbers.length; i++){
            var obj = this.F0.find(f => f.forceType == fNumbers[i]);
            if (typeof(obj) != "undefined"){
                this.F0.splice(this.F0.indexOf(obj), 1);
            } else {
                this.F0.push(new CForcer(fNumbers[i]));
            }
        }
        this.forcerCount = this.F0.length;
    }

    removeAddWall(cNumbers){
        for (var i = 0; i < cNumbers.length; i++){
            var obj = this.C0.find(w => w.wallType == cNumbers[i]);
            if (typeof(obj) != "undefined"){  // remove
                this.C0.splice(this.C0.indexOf(obj), 1);
            } else {  // add
                this.C0.push(new CWall(cNumbers[i]));
            }
        }
        this.wallCount = this.C0.length;
    }
    
}

function distance3D(p1, p2){  // Calculate distance between two 3d points
    var x = Math.abs(p1[0] - p2[0]);
    var y = Math.abs(p1[1] - p2[1]);
    var z = Math.abs(p1[2] - p2[2]);

    return Math.sqrt(x*x + y*y + z*z);
}

function distance2D(p1, p2){  // Calculate distance between two 3d points
    var x = Math.abs(p1[0] - p2[0]);
    var y = Math.abs(p1[1] - p2[1]);

    return Math.sqrt(x*x + y*y);
}

function dot(v1, v2){
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}