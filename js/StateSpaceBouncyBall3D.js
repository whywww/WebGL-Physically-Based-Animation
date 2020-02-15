var gl;
var g_canvas;

// conditionals
control0 = true;
control1 = false;
control2 = false;
control3 = false;
control4 = true;
var hasWall = true;
var hasBall = true;

worldBox = new VBObox1();
partBox1 = new VBObox2();
partBox2 = new VBObox3();
partBox3 = new VBObox4();
partBox4 = new VBObox5();

var g_timeStep = 1000.0/100.0;  // milliseconds
var g_timeStepMin = g_timeStep;
var g_timeStepMax = g_timeStep;
var g_stepCount = 0;

var g_last = Date.now();

// View & Projection
var eyeX = 0.0;
var eyeY = 5.5;
var eyeZ = 1.0;
var atX = 0.0;
var atY = 0;
var atZ = 0.8;
var theta = 0.0;  // turn camera horizontally to angle theta
var r = eyeY-atY;  // radius of camera cylinder
var tilt = 0.0;

var isClear = 1;
var runMode = 3;  // 0: reset; 1: pause; 2: step; 3: run

// Mouse click and drag
var isDrag=false;
var xMclik=0.0;
var yMclik=0.0;   
var xMdragTot=0;
var yMdragTot=0; 


function main() {
    g_canvas = document.getElementById('webgl');
    gl = g_canvas.getContext("webgl", { preserveDrawingBuffer: true});

    if (!gl) {
        console.log('Failed to get the rendering context for WebGL');
        return;
    }

    worldBox.init(gl);
    partBox1.init(gl);
    partBox2.init(gl);
    partBox3.init(gl);
    partBox4.init(gl);

    // Event register
    window.addEventListener("mousedown", myMouseDown);
    window.addEventListener("mousemove", myMouseMove);
    window.addEventListener("mouseup", myMouseUp);
    window.addEventListener("keydown", myKeyDown, false);
    
    // Dat.gui
    var GUIContent = function() {
        this.solver = 'Explicit-Midpoint';
        this.hasWalls = hasWall;
        this.wallElasticity = Kbouncy;
        this.hasGreenBall = hasBall;
        this.greenBallRadius = pBallRadius;
    };
    var BallContent = function(){
        this.toggleMe = function(){control1 = !control1;};
        this.tornado = false;
        this.wind = false;
        this.windVelocity = windVel[0];
        this.boid = false;
    };
    var FireContent = function(){
        this.toggleMe = function(){control4 = !control4;};
    };
    var SpringContent = function(){
        this.springLength = springLen;
        this.stiffness = springStiffness;
        this.damping = springDamp;
        this.gravity = false;
        this.mass = sprMass;
    };
    var SnakeContent = function(){
        this.toggleMe = function(){control2 = !control2;};
        this.toggleFixedPoint = function(){isFixed = !isFixed;};
        this.number = partBox2.pSys.partCount;
    };
    var TetContent = function(){
        this.toggleMe = function(){control3 = !control3;}
    };

    var text = new GUIContent();
    var balltext = new BallContent();
    var fireText = new FireContent();
    var springText = new SpringContent();
    var snakeText = new SnakeContent();
    var tetText = new TetContent();
    var gui = new dat.GUI();


    gui.add(text, 'solver', ['Euler', 'Implicit-Euler', 'Explicit-Midpoint', 'Iter-Implicit-Euler', 'Iter-Implicit-Midpoint', 'Adams-Bashforth', 'Velocity-Verlet', 'Semi-Implicit']).onChange(function(val){
        if (val == 'Euler') solverType = SOLV_EULER;
        else if (val == 'Implicit-Euler') solverType = SOLV_IM_EULER;
        else if (val == 'Explicit-Midpoint') solverType = SOLV_EX_MIDPOINT;
        else if (val == 'Iter-Implicit-Euler') solverType = SOLV_ITER_IM_EULER;
        else if (val == 'Iter-Implicit-Midpoint') solverType = SOLV_ITER_IM_MIDPOINT;
        else if (val == 'Adams-Bashforth') solverType = SOLV_ADAMS_BASHFORTH;
        else if (val == 'Velocity-Verlet') solverType = SOLV_VEL_VERLET;
        else if (val == 'Semi-Implicit') solverType = SOLV_ME;
    }).listen();
    gui.add(text, 'hasWalls').onChange(function(val){
        hasWall = val;
        partBox1.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
        partBox2.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
        partBox3.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
        partBox4.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
    });
    gui.add(text, 'wallElasticity').min(0).max(1).onChange(function(val){Kbouncy = val;});
    gui.add(text, 'hasGreenBall').onChange(function(val){
        hasBall = val;
        partBox1.pSys.removeAddWall([WTYPE_PBALL]);
        partBox2.pSys.removeAddWall([WTYPE_PBALL]);
        partBox3.pSys.removeAddWall([WTYPE_PBALL]);
        partBox4.pSys.removeAddWall([WTYPE_PBALL]);
    });
    gui.add(text, 'greenBallRadius').onChange(function(val){pBallRadius = val;});

    var ball = gui.addFolder('Ball');
    ball.add(balltext, 'toggleMe');
    ball.add(balltext, 'boid').onChange(function(){partBox1.pSys.removeAddForce([F_FLOCK, F_GRAV_E, F_DRAG]);});
    ball.add(balltext, 'tornado').onChange(function(){partBox1.pSys.removeAddForce([F_TORNADO]); partBox1.pSys.removeAddWall([WTYPE_TORNADO])});
    ball.add(balltext, 'wind').onChange(function(){partBox1.pSys.removeAddForce([F_WIND]);});
    ball.add(balltext, 'windVelocity').onChange(function(val){windVel[0] = val;});
    ball.open();

    var fire = gui.addFolder('Fire');
    fire.add(fireText, 'toggleMe');
    fire.open();

    var spring = gui.addFolder('Spring');
    spring.add(springText, 'springLength').onChange(function(val){springLen = val});
    spring.add(springText, 'stiffness').onChange(function(val){springStiffness = val});
    spring.add(springText, 'damping', 0.01, 1).onChange(function(val){springDamp = val});
    spring.add(springText, 'gravity').onChange(function(){partBox2.pSys.removeAddForce([F_DRAG, F_GRAV_E]); partBox3.pSys.removeAddForce([F_DRAG, F_GRAV_E]);});
    spring.add(springText, 'mass').onChange(function(val){sprMass = val;})
    spring.open();

    var sprSnake = spring.addFolder('springSnake');
    sprSnake.add(snakeText, 'toggleMe');
    sprSnake.add(snakeText, 'toggleFixedPoint');
    sprSnake.add(snakeText, 'number').min(1).max(10).step(1).onChange(function(val){partBox2.pSys.partCount = val;});
    sprSnake.open();

    var sprTet = spring.addFolder('SpringTetrahedron');
    sprTet.add(tetText, 'toggleMe');
    sprTet.open();

    gl.clearColor(0.3, 0.3, 0.3, 1);
    gl.enable(gl.DEPTH_TEST); 
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    gl.enable( gl.BLEND );
    gl.blendFunc( gl.SRC_COLOR, gl.ONE_MINUS_SRC_ALPHA );

    vpMatrix = new Matrix4();

    var tick = function() {
        g_timeStep = animate();
        drawResize();
        requestAnimationFrame(tick, g_canvas);
    };
    tick();
}


function animate() {
    var now = Date.now();	
    var elapsed = now - g_last;	
    g_last = now;
    g_stepCount = (g_stepCount +1)%1000;
    if (elapsed < g_timeStepMin) g_timeStepMin = elapsed;
    else if (elapsed > g_timeStepMax) g_timeStepMax = elapsed;
    return elapsed;
}


function drawResize(){
    var nuCanvas = document.getElementById('webgl');	// get current canvas
    var nuGl = getWebGLContext(nuCanvas);
    
    nuCanvas.width = innerWidth - 16;
    nuCanvas.height = innerHeight*3/4 - 16;

    drawAll(nuGl);
}


function drawAll() {
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // 3D view setup
    gl.viewport(0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight); 
	ratio = gl.drawingBufferWidth/gl.drawingBufferHeight;
	vpMatrix.setPerspective(40, ratio, 1, 100);
    // modelMatrix.setOrtho(-Math.tan(20/180*Math.PI)*33/ratio, Math.tan(20/180*Math.PI)*33/ratio, -Math.tan(20/180*Math.PI)*33, Math.tan(20/180*Math.PI)*33, 1.0, 33.0);  // left, right, bottom, top, near, far
    vpMatrix.lookAt(eyeX,eyeY,eyeZ, atX,atY,atZ, 0.0, 0.0, 1.0);
    
    if (control0){
        worldBox.switchToMe();
        worldBox.adjust();
        worldBox.draw();   
    }
    if (control1){  // bouncy ball
        partBox1.switchToMe();
        partBox1.adjust();
        partBox1.draw();  
    }
    if (control2){  // spring snake
        partBox2.switchToMe();
        partBox2.adjust();
        partBox2.draw();     
    }
    if (control3){  // spring tetrahedron
        partBox3.switchToMe();
        partBox3.adjust();
        partBox3.draw();     
    }
    if (control4){  // fire
        partBox4.switchToMe();
        partBox4.adjust();
        partBox4.draw();     
    }
}


//=================Make objects=====================
function makeGroundGrid() {
	var xcount = 100;
	var ycount = 100;
	var xymax	= 50.0;
	var xColr = new Float32Array([0.8, 0.5, 0.8, 0.5]);
	var yColr = new Float32Array([0.8, 0.5, 0.8, 0.5]);

	gndVerts = new Float32Array(floatsPerVertex*2*(xcount+ycount));
						
	var xgap = xymax/(xcount-1);
	var ygap = xymax/(ycount-1);

	for(v=0, j=0; v<2*xcount; v++, j+= floatsPerVertex) {
		if(v%2==0) {
			gndVerts[j  ] = -xymax + (v  )*xgap;
			gndVerts[j+1] = -xymax;	
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		else {
			gndVerts[j  ] = -xymax + (v-1)*xgap;
			gndVerts[j+1] = xymax;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		gndVerts[j+4] = xColr[0];
		gndVerts[j+5] = xColr[1];
        gndVerts[j+6] = xColr[2];
		gndVerts[j+7] = xColr[3];
	}

	for(v=0; v<2*ycount; v++, j+= floatsPerVertex) {
		if(v%2==0) {
			gndVerts[j  ] = -xymax;
			gndVerts[j+1] = -xymax + (v  )*ygap;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		else {
			gndVerts[j  ] = xymax;
			gndVerts[j+1] = -xymax + (v-1)*ygap;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		gndVerts[j+4] = yColr[0];
		gndVerts[j+5] = yColr[1];
        gndVerts[j+6] = yColr[2];
		gndVerts[j+7] = yColr[3];
	}
}


function makeAxis(){
    axisVerts = new Float32Array([
        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3, 1.0,	// X axis line (origin: gray)
        3.3,  0.0,  0.0, 1.0,		1.0,  0.3,  0.3, 1.0,	// 						 (endpoint: red)
		 
        0.0,  0.0,  0.0, 1.0,       0.3,  0.3,  0.3, 1.0,	// Y axis line (origin: white)
        0.0,  3.3,  0.0, 1.0,		0.3,  1.0,  0.3, 1.0,	//						 (endpoint: green)

        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3, 1.0,	// Z axis line (origin:white)
        0.0,  0.0,  3.3, 1.0,		0.3,  0.3,  1.0, 1.0	//						 (endpoint: blue)
    ]);
}


function makeCube(){
    cubeVerts = new Float32Array([
        0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.5, 0.1,
        0.0, 1.0, 0.0, 1.0, 0.7, 0.7, 0.7, 0.1,
        1.0, 1.0, 0.0, 1.0, 0.2, 0.0, 0.5, 0.1,
        1.0, 0.0, 0.0, 1.0, 0.7, 0.7, 0.7, 0.1
    ]);
}


function makeSpring(){
    var rTube = 0.001;  // radius of the tube we bent to form a torus
    var rBend = 0.1;  // radius of the circle made by tube

    var tubeRings = 23;
    var ringSides = 13;

    var thetaStep = 2*Math.PI/tubeRings;
    var phiHalfStep = Math.PI/ringSides;

    var totCirc = 3;  // How many circles of torus
    var unitHeight = 0.2;  // height of each circle along z axis

    sprVerts = new Float32Array(floatsPerVertex * (2*ringSides*tubeRings) * totCirc);

    for (h = 0,j=0; h < totCirc; h++){

        for(s=0; s<tubeRings; s++) {		// for each 'ring' of the torus:

            for(v=0; v< 2*ringSides; v++, j+=floatsPerVertex) {	// for each vertex in this segment:

                if(v%2==0)	{
                    sprVerts[j  ] = (rBend + rTube*Math.cos((v)*phiHalfStep)) * Math.cos((s)*thetaStep);
                    sprVerts[j+1] = (rBend + rTube*Math.cos((v)*phiHalfStep)) * Math.sin((s)*thetaStep);
                    sprVerts[j+2] = (unitHeight * h) + (s * (unitHeight+rTube*Math.sin((v)*phiHalfStep))/tubeRings);
                    sprVerts[j+3] = 1.0;
                }
                else {
                    sprVerts[j  ] = (rBend + rTube * Math.cos((v-1)*phiHalfStep)) * Math.cos((s+1) * thetaStep);
                    sprVerts[j+1] = (rBend + rTube * Math.cos((v-1)*phiHalfStep)) * Math.sin((s+1)*thetaStep);
                    sprVerts[j+2] = (unitHeight * h) + (s * (unitHeight+rTube*Math.sin((v)*phiHalfStep))/tubeRings);
                    sprVerts[j+3] = 1.0;
                }
                if(v==0 && s!=0) {
                    sprVerts[j+4] = 0.0;
                    sprVerts[j+5] = 0.0;		
                    sprVerts[j+6] = 0.2;		
                }
                else {
                    sprVerts[j+4] = 0;
                    sprVerts[j+5] = 0;
                    sprVerts[j+6] = 0.7;
                }
            }
        }
    }
    startPoint = [sprVerts[0], sprVerts[1], sprVerts[2]];
    endPoint = [sprVerts[sprVerts.length-7], sprVerts[sprVerts.length-6], sprVerts[sprVerts.length-5]];
}


function makeSphere() {
    var slices =12;		
    var sliceVerts	= 21;

    var topColr = new Float32Array([0.0, 0.5, 0.0, 1.0]);
    var botColr = new Float32Array([0.0, 0.7, 0.0, 1.0]);
    var errColr = new Float32Array([0.0, 0.5, 0.0, 1.0]);
    var sliceAngle = Math.PI/slices;	

    sphVerts = new Float32Array(((slices*2*sliceVerts)-2) * floatsPerVertex);
                                
    var cosBot = 0.0;				
    var sinBot = 0.0;				
    var cosTop = 0.0;			
    var sinTop = 0.0;
    var j = 0;					
    var isFirstSlice = 1;		
    var isLastSlice = 0;		
    for(s=0; s<slices; s++) {	
        if(s==0) {
            isFirstSlice = 1;		
            cosBot =  0.0; 		
            sinBot = -1.0;		
        }
        else {					
            isFirstSlice = 0;	
            cosBot = cosTop;
            sinBot = sinTop;
        }						
        cosTop = Math.cos((-Math.PI/2) +(s+1)*sliceAngle); 
        sinTop = Math.sin((-Math.PI/2) +(s+1)*sliceAngle);
        if(s==slices-1) isLastSlice=1;
        for(v=isFirstSlice;    v< 2*sliceVerts-isLastSlice;   v++,j+=floatsPerVertex)
        {					
            if(v%2 ==0) { 
                sphVerts[j  ] = cosBot * Math.cos(Math.PI * v/sliceVerts);	
                sphVerts[j+1] = cosBot * Math.sin(Math.PI * v/sliceVerts);	
                sphVerts[j+2] = sinBot;																			// z
                sphVerts[j+3] = 1.0;																				// w.				
            }
            else {	
                sphVerts[j  ] = cosTop * Math.cos(Math.PI * (v-1)/sliceVerts); 
                sphVerts[j+1] = cosTop * Math.sin(Math.PI * (v-1)/sliceVerts);
                sphVerts[j+2] = sinTop;		
                sphVerts[j+3] = 1.0;	
            }
            if(v==0) { 	
                sphVerts[j+4]=errColr[0]; 
                sphVerts[j+5]=errColr[1]; 
                sphVerts[j+6]=errColr[2];
                sphVerts[j+7]=errColr[3];				
                }
            else if(isFirstSlice==1) {	
                sphVerts[j+4]=botColr[0]; 
                sphVerts[j+5]=botColr[1]; 
                sphVerts[j+6]=botColr[2];	
                sphVerts[j+7]=errColr[3];				
                }
            else if(isLastSlice==1) {
                sphVerts[j+4]=topColr[0]; 
                sphVerts[j+5]=topColr[1]; 
                sphVerts[j+6]=topColr[2];	
                sphVerts[j+7]=topColr[3];	
            }
            else {	
                sphVerts[j+4]= 0.0; 
                sphVerts[j+5]= 0.5;	
                sphVerts[j+6]= 0.0;	
                sphVerts[j+7]= 1.0;	
            }
        }
    }
}   


function drawBall(modelMatrix, u_ModelMatrix){

    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step
        // 1) DotFinder(): Find s0Dot from s0 & f0. 
        pSys.applyAllForces(pSys.S0, pSys.F0);
        pSys.dotMaker(pSys.S0dot, pSys.S0, g_timeStep);

        //2) Solver(): Find s1 from s0 & s0dot
        pSys.solver(g_timeStep, pSys.S0, pSys.S0dot, pSys.S1);

        // 3) Apply all constraints
        pSys.doConstraints(pSys.S1, pSys.S0, pSys.C0);
    
        // 4) Render
        pSys.drawMe(pSys.S0, modelMatrix, u_ModelMatrix);

        // 5) Swap
        [pSys.S0, pSys.S1] = pSys.stateVecSwap(pSys.S0, pSys.S1);
    }
    else{  // paused. Only draw current state
        pSys.drawMe(pSys.S0, modelMatrix, u_ModelMatrix);
    }
}


function drawGroundGrid(modelMatrix, u_ModelMatrix){  
    modelMatrix.scale( 0.1, 0.1, 0.1);	
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.LINES, gndStart/7, gndVerts.length/7);
}


function drawAxis(modelMatrix, u_ModelMatrix){
	modelMatrix.scale(0.1, 0.1, 0.1);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.LINES, axisStart/floatsPerVertex, axisVerts.length/floatsPerVertex);
}


function drawCube(modelMatrix, u_ModelMatrix){
    if (hasWall){

        modelMatrix.rotate(90, 1.0, 0.0, 0.0); // rotate ball axis

        // back
        pushMatrix(modelMatrix);
        modelMatrix.scale(2.0, 1.8, 1.0);
        modelMatrix.translate(-0.5, 0.0, 2.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // right
        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(1.0, 1.8, 5.0);
        modelMatrix.translate(-1.0, 0.0, 0.4);
        modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // left
        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(1.0, 1.8, 5.0);
        modelMatrix.translate(1.0, 0.0, 0.4);
        modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // up
        modelMatrix = popMatrix();
        modelMatrix.scale(2.0, 1.8, 5.0);
        modelMatrix.translate(-0.5, 1.0, -0.6);
        modelMatrix.rotate(90.0, 1.0, 0.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);   
    }
}


function drawSpring(pSys, modelMatrix, u_ModelMatrix){

    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step

        // 1) DotFinder(): Find s0Dot from s0 & f0. 
        pSys.applyAllForces(pSys.S0, pSys.F0);
        pSys.dotMaker(pSys.S0dot, pSys.S0, g_timeStep);

        //2) Solver(): Find s1 from s0 & s0dot
        pSys.solver(g_timeStep, pSys.S0, pSys.S0dot, pSys.S1);

        // 3) Apply all constraints
        pSys.doConstraints(pSys.S1, pSys.S0, pSys.C0);
    
        // 4) Render
        partBox2.isPoint = 0;
        pSys.drawMe(pSys.S0, modelMatrix, u_ModelMatrix, gl.LINE_LOOP);
        partBox2.isPoint = 1; // Points
        gl.drawArrays(gl.POINTS, 0, pSys.partCount);

        // 5) Swap
        [pSys.S0, pSys.S1] = pSys.stateVecSwap(pSys.S0, pSys.S1);
    }
    else{  // paused. Only draw current state
        pSys.drawMe(pSys.S0, modelMatrix, u_ModelMatrix, gl.LINES);
    }
}


function drawSphere(modelMatrix, u_ModelMatrix){
    if (hasBall){
        modelMatrix.translate(pBallCenter[0], pBallCenter[1], pBallCenter[2]);
        modelMatrix.scale(pBallRadius, pBallRadius, pBallRadius);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_STRIP, sphStart/floatsPerVertex, sphVerts.length/floatsPerVertex);
    }
}


//====================Control========================
function myKeyDown(ev) {
    switch(ev.code){
		case "ArrowLeft":
            // camera move left
            eyeX += 0.1*Math.cos(theta*Math.PI/180);
            eyeY += 0.1*Math.sin(theta*Math.PI/180);
            atX += 0.1*Math.cos(theta*Math.PI/180);
            atY += 0.1*Math.sin(theta*Math.PI/180);
            break;

        case "ArrowRight":
            // camera move right
            eyeX -= 0.1*Math.cos(theta*Math.PI/180);
            eyeY -= 0.1*Math.sin(theta*Math.PI/180);
            atX -= 0.1*Math.cos(theta*Math.PI/180);
            atY -= 0.1*Math.sin(theta*Math.PI/180);
            break;

        case "ArrowUp":
            atZ += 0.1;
            eyeZ += 0.1;
            break;
        
        case "ArrowDown":
            atZ -= 0.1;
            eyeZ -= 0.1;
            break;

        case "Equal":
            // camera move foward
            eyeX += 0.1*Math.sin(theta*Math.PI/180);
            atX += 0.1*Math.sin(theta*Math.PI/180); 
            eyeY -= 0.1*Math.cos(theta*Math.PI/180);
            atY -= 0.1*Math.cos(theta*Math.PI/180);
            var tan = (atZ - eyeZ) / (atY - eyeY);
            eyeZ -= 0.1*Math.cos(theta*Math.PI/180) * tan;
            atZ -= 0.1*Math.cos(theta*Math.PI/180) * tan;
            break;
        
        case "Minus":
            // camera move backward
            eyeX -= 0.1*Math.sin(theta*Math.PI/180);
            atX -= 0.1*Math.sin(theta*Math.PI/180); 
            eyeY += 0.1*Math.cos(theta*Math.PI/180);
            atY += 0.1*Math.cos(theta*Math.PI/180);
            var tan = (atZ - eyeZ) / (atY - eyeY);
            eyeZ += 0.1*Math.cos(theta*Math.PI/180) * tan;
            atZ += 0.1*Math.cos(theta*Math.PI/180) * tan;
            break;

        case "KeyW":
            // camera move up
            atZ += 0.1;  // tilt
            break;

        case "KeyS":
            // camera move down
			atZ -= 0.1;  // tilt
            break;

        case "KeyA":
            // camera look left
            theta += 2;
            atX = eyeX + r*Math.sin(theta*Math.PI/180);
            atY = eyeY - r*Math.cos(theta*Math.PI/180);
            break;

        case "KeyD":
            // camera look right
            theta -= 2;
            atX = eyeX + r*Math.sin(theta*Math.PI/180);
            atY = eyeY - r*Math.cos(theta*Math.PI/180);
            break;

        case "KeyR":
            // boost velocity only.
            var pSys = partBox1.pSys;
            for (var i = 0; i < pSys.partCount; i++){
                if (pSys.S0[i * PART_MAXVAR + PART_XVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_XVEL] += (Math.random()+1) * 2;
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_XVEL] -= (Math.random()+1) * 2;
                }
                if (pSys.S0[i * PART_MAXVAR + PART_YVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_YVEL] += (Math.random()+1) * 2;  // Also g_drag should be applied??
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_YVEL] -= (Math.random()+1) * 2;
                }
                if (pSys.S0[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] += (Math.random()+1) * 5;
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] -= (Math.random()+1) * 5;
                }
            }
            break;
        
        case "KeyP":
            if (runMode == 3) runMode = 1;
            else runMode = 3;
            break;

        case "Space":
            runMode = 2;
            break;

        default:
            break;
	}
}

function myMouseDown(ev) {  
    var rect = ev.target.getBoundingClientRect();
    var xp = ev.clientX - rect.left;
    var yp = g_canvas.height - (ev.clientY - rect.top);
    // webgl(CVV) coords
    var x = (xp - g_canvas.width/2) / (g_canvas.width/2);
    var y = (yp - g_canvas.height/2) / (g_canvas.height/2);
    isDrag = true;
    xMclik = x;	
    yMclik = y;

    // dis = [];
    // for (var i = 0, k = 0; i < partBox2.pSys.partCount; i++, k += PART_MAXVAR){
    //     dis.push(distance([-x, 0, (y+1)*0.85], 
    //                         [partBox2.pSys.S0[k + PART_XPOS],
    //                         0,
    //                         partBox2.pSys.S0[k + PART_ZPOS]]));
    // }
    // minDis = dis.indexOf(Math.min(...dis));
}

function myMouseMove(ev){
    if(isDrag==false) return;	

    var rect = ev.target.getBoundingClientRect();	
    var xp = ev.clientX - rect.left;							
    var yp = g_canvas.height - (ev.clientY - rect.top);
    
    var x = (xp - g_canvas.width/2) / (g_canvas.width/2);	
    var y = (yp - g_canvas.height/2) / (g_canvas.height/2);
 
    xMdragTot = (x - xMclik);
    yMdragTot = (y - yMclik);

    pBallCenter[0] -= (x - xMclik);
    pBallCenter[2] += (y - yMclik);
    
    xMclik = x;
    yMclik = y;
}

function myMouseUp(ev) {
    var rect = ev.target.getBoundingClientRect();	
    var xp = ev.clientX - rect.left;							
    var yp = g_canvas.height - (ev.clientY - rect.top);

    var x = (xp - g_canvas.width/2) /	(g_canvas.width/2);		
    var y = (yp - g_canvas.height/2) / (g_canvas.height/2);

    isDrag = false;	
    // xMdragTot -= (x - xMclik);
    // yMdragTot += (y - yMclik);
    // console.log('upppppppp:', xMdragTot);
}