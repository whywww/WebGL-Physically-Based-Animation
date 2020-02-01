var gl;
var g_canvas;

control0 = 1;
control1 = 1;
control2 = 0;

worldBox = new VBObox1();
partBox1 = new VBObox2();
partBox2 = new VBObox3();

var g_timeStep = 1000.0/60.0;  // milliseconds
var g_timeStepMin = g_timeStep;
var g_timeStepMax = g_timeStep;
var g_stepCount = 0;

var g_last = Date.now();

// View & Projection
var eyeX = 0.0;
var eyeY = 4.5;
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

    // Event register
    window.addEventListener("mousedown", myMouseDown);
    window.addEventListener("mousemove", myMouseMove);
    window.addEventListener("mouseup", myMouseUp);
	window.addEventListener("keydown", myKeyDown, false);

    gl.clearColor(0.3, 0.3, 0.3, 1);
    gl.enable(gl.DEPTH_TEST); 
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    vpMatrix = new Matrix4();

    var tick = function() {
        g_timeStep = animate();
        drawAll();
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


function drawAll() {
    // if (isClear){
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    // }

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
    if (control1){
        partBox1.switchToMe();
        partBox1.adjust();
        partBox1.draw();  
    }
    if (control2){
        partBox2.switchToMe();
        partBox2.adjust();
        partBox2.draw();     
    }
}


//=================Make objects=====================
function makeGroundGrid() {
	var xcount = 100;
	var ycount = 100;
	var xymax	= 50.0;
	var xColr = new Float32Array([1.0, 1.0, 0.3]);
	var yColr = new Float32Array([0.5, 1.0, 0.5]);

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
	}
}


function makeAxis(){
    axisVerts = new Float32Array([
        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3,	// X axis line (origin: gray)
        3.3,  0.0,  0.0, 1.0,		1.0,  0.3,  0.3,	// 						 (endpoint: red)
		 
        0.0,  0.0,  0.0, 1.0,       0.3,  0.3,  0.3,	// Y axis line (origin: white)
        0.0,  3.3,  0.0, 1.0,		0.3,  1.0,  0.3,	//						 (endpoint: green)

        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3,	// Z axis line (origin:white)
        0.0,  0.0,  3.3, 1.0,		0.3,  0.3,  1.0,	//						 (endpoint: blue)
    ]);
}


function makeCube(){
    cubeVerts = new Float32Array([
        0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0,
        0.0, 1.0, 0.0, 1.0, 0.3, 0.3, 0.3,
        1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 0.0, 0.0, 1.0, 0.3, 0.3, 0.3,
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
	modelMatrix.rotate(90, 1.0, 0.0, 0.0); // rotate ball axis

    pushMatrix(modelMatrix);
    modelMatrix.scale(2.0, 1.8, 1.0);
    modelMatrix.translate(-0.5, 0.0, 2.0);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

    modelMatrix = popMatrix();
    pushMatrix(modelMatrix);
    modelMatrix.scale(1.0, 1.8, 10.0);
    modelMatrix.translate(-1.0, 0.0, 0.2);
    modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

    modelMatrix = popMatrix();
    pushMatrix(modelMatrix);
    modelMatrix.scale(1.0, 1.8, 10.0);
    modelMatrix.translate(1.0, 0.0, 0.2);
    modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

    modelMatrix = popMatrix();
    modelMatrix.scale(4.0, 1.0, 10.0);
    modelMatrix.translate(-0.5, 1.8, -0.2);
    modelMatrix.rotate(90.0, 1.0, 0.0, 0.0);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLE_FAN, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);   

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

        case "KeyI":
            // camera move up
            atZ += 0.1;  // tilt
            break;

        case "KeyK":
            // camera move down
			atZ -= 0.1;  // tilt
            break;

        case "KeyJ":
            // camera look left
            theta += 2;
            atX = eyeX + r*Math.sin(theta*Math.PI/180);
            atY = eyeY - r*Math.cos(theta*Math.PI/180);
            break;

        case "KeyL":
            // camera look right
            theta -= 2;
            atX = eyeX + r*Math.sin(theta*Math.PI/180);
            atY = eyeY - r*Math.cos(theta*Math.PI/180);
            break;

        case "KeyR":
            // boost velocity only.
            var pSys = partBox1.pSys;
            for (i = 0; i < pSys.partCount; i++){
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
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] += (Math.random()+1) * 2;
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] -= (Math.random()+1) * 2;
                }
            }
            break;
        
        case "KeyC":
            // toggle clear screen
            if (isClear){
                isClear = 0;
            }else{
                isClear = 1;
            }
            break;
        
        case "KeyP":
            if (runMode == 3) runMode = 1;
            else runMode = 3;
            break;

        case "Space":
            runMode = 2;
            break;
        
        case "KeyF":
            // Fountain
            if(isFountain == 0)
                isFountain = 1;
            else
                isFountain = 0;
            break;

        case "KeyX":
            if (control1 & !control2){
                control1 = 0;
                control2 = 1;
            }
            else{
                control1 = 1;
                control2 = 0;
            }
            break;

        case "KeyS":
            // change solver
            solverType++;
            if (solverType >= SOLV_MAX){
                solverType = 0;
            } 
            console.log(solverType);
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
}

function myMouseMove(ev){
    if(isDrag==false) return;	

    var rect = ev.target.getBoundingClientRect();	
    var xp = ev.clientX - rect.left;							
    var yp = g_canvas.height - (ev.clientY - rect.top);
    
    var x = (xp - g_canvas.width/2) / (g_canvas.width/2);	
    var y = (yp - g_canvas.height/2) / (g_canvas.height/2);
 
    xMdragTot += (x - xMclik);
    yMdragTot += (y - yMclik);
    
    if (control2){
        var pSys = partBox2.pSys;
        pSys.S0[PART_MAXVAR + PART_XPOS] += (x - xMclik);
        pSys.S0[PART_MAXVAR + PART_ZPOS] += (y - yMclik);  
    }
    
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
    xMdragTot += (x - xMclik);
    yMdragTot += (y - yMclik);
    
}