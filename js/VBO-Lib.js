var floatsPerVertex = 8;
var flameSize = 5;

/****************** VBObox1 ***********************/
// Rigid Objects
function VBObox1(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec4 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = a_Color; \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    'varying vec4 v_Color; \n' +
    'void main() {\n' +
    '   gl_FragColor = v_Color; \n' +
    '}\n';

    makeGroundGrid();
    makeAxis();
    makeCube();
    makeSphere();

    var totalSize = gndVerts.length + axisVerts.length 
                    + cubeVerts.length + sphVerts.length;
    var vcount = totalSize/floatsPerVertex;
    vertices = new Float32Array(totalSize);	

    gndStart = 0;
    for (i = 0, j = 0; j < gndVerts.length; i++, j++){
        vertices[i] = gndVerts[j];
    }
    axisStart = i;
    for(j=0; j< axisVerts.length; i++, j++) {
        vertices[i] = axisVerts[j];
    }
    cubeStart = i;
    for(j=0; j< cubeVerts.length; i++, j++) {
        vertices[i] = cubeVerts[j];
    }
    sphStart = i;
    for(j=0; j< sphVerts.length; i++, j++) {
        vertices[i] = sphVerts[j];
    }

    this.vboContents = vertices;
    this.vboVerts = vcount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos =  4;
    this.vboFcount_a_Colr = 4;
    
    console.assert((this.vboFcount_a_Pos + 
        this.vboFcount_a_Colr) * 
        this.FSIZE == this.vboStride, 
        "Uh oh! VBObox1.vboStride disagrees with attribute-size values!");

    this.vboOffset_a_Pos = 0;
    this.vboOffset_a_Colr = this.vboFcount_a_Pos * this.FSIZE;

    this.vboLoc;
    this.shaderLoc;	
    this.a_PosLoc;	
    this.a_ColrLoc;	
    this.ModelMat = new Matrix4();
    this.u_ModelMatLoc;	
}

VBObox1.prototype.init = function(){
    this.shaderLoc = createProgram(gl, this.VERT_SRC, this.FRAG_SRC);
	if (!this.shaderLoc) {
        console.log(this.constructor.name + 
                                '.init() failed to create executable Shaders on the GPU. Bye!');
        return;
    }
    gl.program = this.shaderLoc;
    
    // b) Create VBO on GPU, fill it
    this.vboLoc = gl.createBuffer();	
    if (!this.vboLoc) {
        console.log(this.constructor.name + 
    						'.init() failed to create VBO in GPU. Bye!'); 
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ARRAY_BUFFER, this.vboContents, gl.STATIC_DRAW);  

    // c) Find GPU locations for vars 
    this.a_PosLoc = gl.getAttribLocation(this.shaderLoc, 'a_Position');
    if(this.a_PosLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() Failed to get GPU location of attribute a_Pos0');
        return -1;
    }
    this.a_ColrLoc = gl.getAttribLocation(this.shaderLoc, 'a_Color');
    if(this.a_ColrLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_Colr0');
        return -1;
    }

    this.u_ModelMatLoc = gl.getUniformLocation(this.shaderLoc, 'u_ModelMatrix');
    if (!this.u_ModelMatLoc) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_ModelMat1 uniform');
        return;
    }
}

VBObox1.prototype.switchToMe = function() {
    gl.useProgram(this.shaderLoc);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);
    gl.vertexAttribPointer( this.a_PosLoc,
                            this.vboFcount_a_Pos,
                            gl.FLOAT,
                            false,
                            this.vboStride,
                            this.vboOffset_a_Pos);	
    gl.vertexAttribPointer( this.a_ColrLoc, this.vboFcount_a_Colr, 
                            gl.FLOAT, false, 
                            this.vboStride, this.vboOffset_a_Colr );
                                
    gl.enableVertexAttribArray(this.a_PosLoc);
    gl.enableVertexAttribArray(this.a_ColrLoc);
}

VBObox1.prototype.adjust = function() {
    if(this.isReady()==false) {
        console.log('ERROR! before' + this.constructor.name + 
                            '.adjust() call you needed to call this.switchToMe()!!');
    }
    this.ModelMat.set(vpMatrix);
}

VBObox1.prototype.draw = function() {
    pushMatrix(this.ModelMat);
    drawAxis(this.ModelMat, this.u_ModelMatLoc);
    
	this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
    drawCube(this.ModelMat, this.u_ModelMatLoc);
    
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
    drawGroundGrid(this.ModelMat, this.u_ModelMatLoc);
    
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
	drawSphere(this.ModelMat, this.u_ModelMatLoc);
}

VBObox1.prototype.isReady = function() {
    var isOK = true;

    if(gl.getParameter(gl.CURRENT_PROGRAM) != this.shaderLoc)  {
        console.log(this.constructor.name + 
                            '.isReady() false: shader program at this.shaderLoc not in use!');
        isOK = false;
    }
    if(gl.getParameter(gl.ARRAY_BUFFER_BINDING) != this.vboLoc) {
        console.log(this.constructor.name + 
                            '.isReady() false: vbo at this.vboLoc not in use!');
        isOK = false;
    }
    return isOK;
}

/****************** VBObox2 ***********************/
// Particle Systems. Bouncy Balls.
function VBObox2(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec4 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 8.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = a_Color; \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    'varying vec4 v_Color; \n' +
    'void main() {\n' +
    '		float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '		if(dist < 0.5) { \n' +	
    '			gl_FragColor = v_Color;\n' +
    '    	}\n' +
    '		else { \n' + 
    '			discard;\n' +
    '		}\n' +
    '}\n';
    // initiate particle system
    this.pSys = new CPartSys();
    this.partCount = 600;
    this.forces = [F_NONE, F_GRAV_E, F_DRAG];
    this.walls = [WTYPE_GROUND, WTYPE_PBALL, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    this.pSys.initBouncyBall(this.partCount, this.forces, this.walls);

    this.vboContents = this.pSys.S0;
    this.vboVerts = this.pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos = 4;
    this.vboFcount_a_Colr = 4;

    console.assert(PART_MAXVAR * this.FSIZE == this.vboStride, 
        "Uh oh! VBObox2.vboStride disagrees with attribute-size values!");

    this.vboOffset_a_Pos = PART_XPOS * this.FSIZE;
    this.vboOffset_a_Colr = PART_R * this.FSIZE;

    this.vboLoc;
    this.shaderLoc;	
    this.a_PosLoc;	
    this.a_ColrLoc;	
    this.ModelMat = new Matrix4();
    this.u_ModelMatLoc;	
}

VBObox2.prototype.init = function(){
    this.shaderLoc = createProgram(gl, this.VERT_SRC, this.FRAG_SRC);
	if (!this.shaderLoc) {
        console.log(this.constructor.name + 
                                '.init() failed to create executable Shaders on the GPU. Bye!');
        return;
    }
    gl.program = this.shaderLoc;
    
    // b) Create VBO on GPU, fill it
    this.vboLoc = gl.createBuffer();	
    if (!this.vboLoc) {
        console.log(this.constructor.name + 
    						'.init() failed to create VBO in GPU. Bye!'); 
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ARRAY_BUFFER, this.vboContents, gl.DYNAMIC_DRAW);  

    // c) Find GPU locations for vars 
    this.a_PosLoc = gl.getAttribLocation(this.shaderLoc, 'a_Position');
    if(this.a_PosLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() Failed to get GPU location of attribute a_Pos0');
        return -1;
    }
    this.a_ColrLoc = gl.getAttribLocation(this.shaderLoc, 'a_Color');
    if(this.a_ColrLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_Colr0');
        return -1;
    }

    this.u_ModelMatLoc = gl.getUniformLocation(this.shaderLoc, 'u_ModelMatrix');
    if (!this.u_ModelMatLoc) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_ModelMat1 uniform');
        return;
    }
}

VBObox2.prototype.switchToMe = function() {
    gl.useProgram(this.shaderLoc);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);
    gl.vertexAttribPointer( this.a_PosLoc,
                            this.vboFcount_a_Pos,
                            gl.FLOAT,
                            false,
                            this.vboStride,
                            this.vboOffset_a_Pos);	
    gl.vertexAttribPointer( this.a_ColrLoc, 
                            this.vboFcount_a_Colr, 
                            gl.FLOAT, 
                            false, 
                            this.vboStride, 
                            this.vboOffset_a_Colr);
                                
    gl.enableVertexAttribArray(this.a_PosLoc);
    gl.enableVertexAttribArray(this.a_ColrLoc);
}

VBObox2.prototype.adjust = function() {
    if(this.isReady()==false) {
        console.log('ERROR! before' + this.constructor.name + 
                            '.adjust() call you needed to call this.switchToMe()!!');
    }
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
}

VBObox2.prototype.draw = function() {
    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step
         // 1) DotFinder(): Find s0Dot from s0 & f0. 
         this.pSys.applyAllForces(this.pSys.S0, this.pSys.F0);
         this.pSys.dotMaker(this.pSys.S0dot, this.pSys.S0, g_timeStep);
 
         //2) Solver(): Find s1 from s0 & s0dot
         this.pSys.solver(g_timeStep, this.pSys.S0, this.pSys.S0dot, this.pSys.S1);
 
         // 3) Apply all constraints
         this.pSys.doConstraints(this.pSys.S1, this.pSys.S0, this.pSys.C0);
     
         // 4) Render
         this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);

        // 5) Swap
        [this.pSys.S0, this.pSys.S1] = this.pSys.stateVecSwap(this.pSys.S0, this.pSys.S1);

    }
    else{  // paused. Only draw current state
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);
    }
}

VBObox2.prototype.isReady = function() {
    var isOK = true;

    if(gl.getParameter(gl.CURRENT_PROGRAM) != this.shaderLoc)  {
        console.log(this.constructor.name + 
                            '.isReady() false: shader program at this.shaderLoc not in use!');
        isOK = false;
    }
    if(gl.getParameter(gl.ARRAY_BUFFER_BINDING) != this.vboLoc) {
        console.log(this.constructor.name + 
                            '.isReady() false: vbo at this.vboLoc not in use!');
        isOK = false;
    }
    return isOK;
}


/****************** VBObox3 ***********************/
// Spring Snake
function VBObox3(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec4 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = a_Color; \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    
    'varying vec4 v_Color; \n' +
    'uniform int u_isPoint; \n' +

    'void main() {\n' +
    '   u_isPoint;\n'+
    '   if (u_isPoint == 1){ \n' +
    '		float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '		if(dist < 0.5) { \n' +	
    '			gl_FragColor = vec4((1.0-2.0*dist)*v_Color.rgb, 1.0);\n' +
    '    	}\n' +
    '		else { \n' + 
    '			discard;\n' +
    '		}\n' +
    '   }\n' +
    '   else if (u_isPoint == 0) {\n'+
    '       gl_FragColor = v_Color;\n' +
    '   }\n'+
    '}\n';

    // initiate spring system
    this.pSys = new CPartSys();
    this.partCount = 10;
    this.forces = [F_NONE, F_SPRING_SNAKE];
    this.walls = [WTYPE_PBALL, WTYPE_GROUND, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    this.pSys.initSpringSnake(this.partCount, this.forces, this.walls);

    this.vboContents = this.pSys.S0;
    this.vboVerts = this.pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos = 4;
    this.vboFcount_a_Colr = 4;

    console.assert(PART_MAXVAR * this.FSIZE == this.vboStride, 
        "Uh oh! VBObox3.vboStride disagrees with attribute-size values!");

    this.vboOffset_a_Pos = PART_XPOS * this.FSIZE;
    this.vboOffset_a_Colr = PART_R * this.FSIZE;

    this.vboLoc;
    this.shaderLoc;	
    this.a_PosLoc;	
    this.a_ColrLoc;	
    this.ModelMat = new Matrix4();
    this.u_ModelMatLoc;	
}

VBObox3.prototype.init = function(){
    this.shaderLoc = createProgram(gl, this.VERT_SRC, this.FRAG_SRC);
	if (!this.shaderLoc) {
        console.log(this.constructor.name + 
                                '.init() failed to create executable Shaders on the GPU. Bye!');
        return;
    }
    gl.program = this.shaderLoc;
    
    // b) Create VBO on GPU, fill it
    this.vboLoc = gl.createBuffer();	
    if (!this.vboLoc) {
        console.log(this.constructor.name + 
    						'.init() failed to create VBO in GPU. Bye!'); 
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ARRAY_BUFFER, this.vboContents, gl.DYNAMIC_DRAW);  

    // c) Find GPU locations for vars 
    this.a_PosLoc = gl.getAttribLocation(this.shaderLoc, 'a_Position');
    if(this.a_PosLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() Failed to get GPU location of attribute a_Pos');
        return -1;
    }
    this.a_ColrLoc = gl.getAttribLocation(this.shaderLoc, 'a_Color');
    if(this.a_ColrLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_Colr');
        return -1;
    }

    this.u_ModelMatLoc = gl.getUniformLocation(this.shaderLoc, 'u_ModelMatrix');
    if (!this.u_ModelMatLoc) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_ModelMat uniform');
        return;
    }

    this.u_isPoint = gl.getUniformLocation(this.shaderLoc, 'u_isPoint');
    if (!this.u_isPoint) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_isPoint uniform');
        return;
    }
}

VBObox3.prototype.switchToMe = function() {
    gl.useProgram(this.shaderLoc);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);
    gl.vertexAttribPointer( this.a_PosLoc,
                            this.vboFcount_a_Pos,
                            gl.FLOAT,
                            false,
                            this.vboStride,
                            this.vboOffset_a_Pos);	
    gl.vertexAttribPointer( this.a_ColrLoc, 
                            this.vboFcount_a_Colr, 
                            gl.FLOAT, 
                            false, 
                            this.vboStride, 
                            this.vboOffset_a_Colr);
                                
    gl.enableVertexAttribArray(this.a_PosLoc);
    gl.enableVertexAttribArray(this.a_ColrLoc);
}

VBObox3.prototype.adjust = function() {
    if(this.isReady()==false) {
        console.log('ERROR! before' + this.constructor.name + 
                            '.adjust() call you needed to call this.switchToMe()!!');
    }
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
}

VBObox3.prototype.draw = function() {
    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step

        // 1) DotFinder(): Find s0Dot from s0 & f0. 
        this.pSys.applyAllForces(this.pSys.S0, this.pSys.F0);
        this.pSys.dotMaker(this.pSys.S0dot, this.pSys.S0, g_timeStep);

        //2) Solver(): Find s1 from s0 & s0dot
        this.pSys.solver(g_timeStep, this.pSys.S0, this.pSys.S0dot, this.pSys.S1);

        // 3) Apply all constraints
        this.pSys.doConstraints(this.pSys.S1, this.pSys.S0, this.pSys.C0);
    
        // 4) Render
        gl.uniform1i(this.u_isPoint, 1);
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);
        gl.uniform1i(this.u_isPoint, 0);
        gl.drawArrays(gl.LINE_STRIP, 0, this.pSys.partCount);

        // 5) Swap
        [this.pSys.S0, this.pSys.S1] = this.pSys.stateVecSwap(this.pSys.S0, this.pSys.S1);
    }
    else{  // paused. Only draw current state
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc, gl.LINES);
    }
}

VBObox3.prototype.isReady = function() {
    var isOK = true;

    if(gl.getParameter(gl.CURRENT_PROGRAM) != this.shaderLoc)  {
        console.log(this.constructor.name + 
                            '.isReady() false: shader program at this.shaderLoc not in use!');
        isOK = false;
    }
    if(gl.getParameter(gl.ARRAY_BUFFER_BINDING) != this.vboLoc) {
        console.log(this.constructor.name + 
                            '.isReady() false: vbo at this.vboLoc not in use!');
        isOK = false;
    }
    return isOK;
}


/****************** VBObox4 ***********************/
// Spring Tetrahedron
function VBObox4(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec4 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = a_Color; \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    
    'varying vec4 v_Color; \n' +
    'uniform int u_isPoint; \n' +

    'void main() {\n' +
    '   u_isPoint;\n'+
    '   if (u_isPoint == 1){ \n' +
    '		float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '		if(dist < 0.5) { \n' +	
    '			gl_FragColor = vec4((1.0-2.0*dist)*v_Color.rgb, 1.0);\n' +
    '    	}\n' +
    '		else { \n' + 
    '			discard;\n' +
    '		}\n' +
    '   }\n' +
    '   else if (u_isPoint == 0) {\n'+
    '       gl_FragColor = v_Color;\n' +
    '   }\n'+
    '}\n';

    this.indices = new Uint8Array([
        0, 2,
        1, 3
    ]);

    // initiate spring system
    this.pSys = new CPartSys();
    this.partCount = 4;
    this.forces = [F_NONE, F_SPRING_TET];
    this.walls = [WTYPE_PBALL, WTYPE_GROUND, WTYPE_YWALL_LO, WTYPE_YWALL_HI, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    this.pSys.initSpringTet(this.partCount, this.forces, this.walls);

    this.vboContents = this.pSys.S0;
    this.vboVerts = this.pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos = 4;
    this.vboFcount_a_Colr = 4;

    console.assert(PART_MAXVAR * this.FSIZE == this.vboStride, 
        "Uh oh! VBObox4.vboStride disagrees with attribute-size values!");

    this.vboOffset_a_Pos = PART_XPOS * this.FSIZE;
    this.vboOffset_a_Colr = PART_R * this.FSIZE;

    this.vboLoc;
    this.shaderLoc;	
    this.a_PosLoc;	
    this.a_ColrLoc;	
    this.ModelMat = new Matrix4();
    this.u_ModelMatLoc;	
}

VBObox4.prototype.init = function(){
    this.shaderLoc = createProgram(gl, this.VERT_SRC, this.FRAG_SRC);
	if (!this.shaderLoc) {
        console.log(this.constructor.name + 
                                '.init() failed to create executable Shaders on the GPU. Bye!');
        return;
    }
    gl.program = this.shaderLoc;
    
    // b) Create VBO on GPU, fill it
    this.vboLoc = gl.createBuffer();	
    if (!this.vboLoc) {
        console.log(this.constructor.name + 
    						'.init() failed to create VBO in GPU. Bye!'); 
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ARRAY_BUFFER, this.vboContents, gl.DYNAMIC_DRAW);  

    // element buffer
    this.vboEleLoc = gl.createBuffer();	
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.vboEleLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, this.indices, gl.DYNAMIC_DRAW);  

    // c) Find GPU locations for vars 
    this.a_PosLoc = gl.getAttribLocation(this.shaderLoc, 'a_Position');
    if(this.a_PosLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() Failed to get GPU location of attribute a_Pos');
        return -1;
    }
    this.a_ColrLoc = gl.getAttribLocation(this.shaderLoc, 'a_Color');
    if(this.a_ColrLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_Colr');
        return -1;
    }

    this.u_ModelMatLoc = gl.getUniformLocation(this.shaderLoc, 'u_ModelMatrix');
    if (!this.u_ModelMatLoc) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_ModelMat uniform');
        return;
    }

    this.u_isPoint = gl.getUniformLocation(this.shaderLoc, 'u_isPoint');
    if (!this.u_isPoint) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_isPoint uniform');
        return;
    }
}

VBObox4.prototype.switchToMe = function() {
    gl.useProgram(this.shaderLoc);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);
    gl.vertexAttribPointer( this.a_PosLoc,
                            this.vboFcount_a_Pos,
                            gl.FLOAT,
                            false,
                            this.vboStride,
                            this.vboOffset_a_Pos);	
    gl.vertexAttribPointer( this.a_ColrLoc, 
                            this.vboFcount_a_Colr, 
                            gl.FLOAT, 
                            false, 
                            this.vboStride, 
                            this.vboOffset_a_Colr);
                                
    gl.enableVertexAttribArray(this.a_PosLoc);
    gl.enableVertexAttribArray(this.a_ColrLoc);
}

VBObox4.prototype.adjust = function() {
    if(this.isReady()==false) {
        console.log('ERROR! before' + this.constructor.name + 
                            '.adjust() call you needed to call this.switchToMe()!!');
    }
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
}

VBObox4.prototype.draw = function() {
    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step

        // 1) DotFinder(): Find s0Dot from s0 & f0. 
        this.pSys.applyAllForces(this.pSys.S0, this.pSys.F0);
        this.pSys.dotMaker(this.pSys.S0dot, this.pSys.S0, g_timeStep);

        //2) Solver(): Find s1 from s0 & s0dot
        this.pSys.solver(g_timeStep, this.pSys.S0, this.pSys.S0dot, this.pSys.S1);

        // 3) Apply all constraints
        this.pSys.doConstraints(this.pSys.S1, this.pSys.S0, this.pSys.C0);
    
        // 4) Render
        gl.uniform1i(this.u_isPoint, 1);
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);
        gl.uniform1i(this.u_isPoint, 0);
        gl.drawArrays(gl.LINE_LOOP, 0, this.pSys.partCount);
        gl.drawElements(gl.LINE_STRIP, 4, gl.UNSIGNED_BYTE, 0);

        // 5) Swap
        [this.pSys.S0, this.pSys.S1] = this.pSys.stateVecSwap(this.pSys.S0, this.pSys.S1);
    }
    else{  // paused. Only draw current state
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc, gl.LINES);
    }
}

VBObox4.prototype.isReady = function() {
    var isOK = true;

    if(gl.getParameter(gl.CURRENT_PROGRAM) != this.shaderLoc)  {
        console.log(this.constructor.name + 
                            '.isReady() false: shader program at this.shaderLoc not in use!');
        isOK = false;
    }
    if(gl.getParameter(gl.ARRAY_BUFFER_BINDING) != this.vboLoc) {
        console.log(this.constructor.name + 
                            '.isReady() false: vbo at this.vboLoc not in use!');
        isOK = false;
    }
    return isOK;
}

/****************** VBObox5 ***********************/
// Fire
function VBObox5(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec4 a_Color;\n' +
    'attribute float a_PointSize; \n' + 

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = a_PointSize;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = a_Color; \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    'varying vec4 v_Color; \n' +
    'void main() {\n' +
    '	float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '	if(dist < 0.5) { \n' +	
    '		gl_FragColor = v_Color;\n' +
    '    }\n' +
    '	else { \n' + 
    '		discard;\n' +
    '	}\n' +
    '}\n';

    // initiate particle system
    this.pSys = new CPartSys();
    this.partCount = 600;
    this.forces = [F_NONE, F_GRAV_E, F_DRAG];
    this.walls = [WTYPE_FIRE, WTYPE_GROUND, WTYPE_PBALL, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    this.pSys.initBouncyBall(this.partCount, this.forces, this.walls);

    // this.vertices = new Float32Array(2*this.pSys.S0.length);
    // this.vertices.set(this.pSys.S0);
    // this.vertices.set(this.pSys.S1, this.pSys.S0.length);

    // this.vboContents = this.vertices;
    this.vboContents = this.pSys.S0;
    this.vboVerts = this.pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes/this.vboVerts;
    // this.vboStride = this.pSys.S0.length * this.FSIZE; 
    
    this.vboFcount_a_Pos = 4;
    this.vboFcount_a_Colr = 4;
    this.vboFcount_a_PointSize = 1;

    console.assert(PART_MAXVAR * this.FSIZE == this.vboStride, 
        "Uh oh! VBObox5.vboStride disagrees with attribute-size values!");

    this.vboOffset_a_Pos = PART_XPOS * this.FSIZE;
    this.vboOffset_a_Colr = PART_R * this.FSIZE;
    this.vboOffset_a_PointSize = PART_SIZE * this.FSIZE;

    this.vboLoc;
    this.shaderLoc;	
    this.a_PosLoc;	
    this.a_ColrLoc;	
    this.ModelMat = new Matrix4();
    this.u_ModelMatLoc;	
}

VBObox5.prototype.init = function(){
    this.shaderLoc = createProgram(gl, this.VERT_SRC, this.FRAG_SRC);
	if (!this.shaderLoc) {
        console.log(this.constructor.name + 
                                '.init() failed to create executable Shaders on the GPU. Bye!');
        return;
    }
    gl.program = this.shaderLoc;
    
    // b) Create VBO on GPU, fill it
    this.vboLoc = gl.createBuffer();	
    if (!this.vboLoc) {
        console.log(this.constructor.name + 
    						'.init() failed to create VBO in GPU. Bye!'); 
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);  // Specify purpose of the VBO
    gl.bufferData(gl.ARRAY_BUFFER, this.vboContents, gl.DYNAMIC_DRAW);  

    // c) Find GPU locations for vars 
    this.a_PosLoc = gl.getAttribLocation(this.shaderLoc, 'a_Position');
    if(this.a_PosLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() Failed to get GPU location of attribute a_Pos');
        return -1;
    }
    this.a_ColrLoc = gl.getAttribLocation(this.shaderLoc, 'a_Color');
    if(this.a_ColrLoc < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_Colr');
        return -1;
    }

    this.a_PointSize = gl.getAttribLocation(this.shaderLoc, 'a_PointSize');
    if(this.a_PointSize < 0) {
        console.log(this.constructor.name + 
                              '.init() failed to get the GPU location of attribute a_PointSize');
        return -1;
    }

    this.u_ModelMatLoc = gl.getUniformLocation(this.shaderLoc, 'u_ModelMatrix');
    if (!this.u_ModelMatLoc) { 
        console.log(this.constructor.name + 
                              '.init() failed to get GPU location for u_ModelMat uniform');
        return;
    }
}

VBObox5.prototype.switchToMe = function() {

    gl.useProgram(this.shaderLoc);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vboLoc);
    gl.vertexAttribPointer( this.a_PosLoc,
                            this.vboFcount_a_Pos,
                            gl.FLOAT,
                            false,
                            this.vboStride,
                            this.vboOffset_a_Pos);	
    gl.vertexAttribPointer( this.a_ColrLoc, 
                            this.vboFcount_a_Colr, 
                            gl.FLOAT, 
                            false, 
                            this.vboStride, 
                            this.vboOffset_a_Colr);
    gl.vertexAttribPointer( this.a_PointSize, 
                            this.vboFcount_a_PointSize, 
                            gl.FLOAT, 
                            false, 
                            this.vboStride, 
                            this.vboOffset_a_PointSize); 
    gl.enableVertexAttribArray(this.a_PosLoc);
    gl.enableVertexAttribArray(this.a_ColrLoc);
    gl.enableVertexAttribArray(this.a_PointSize);
}

VBObox5.prototype.adjust = function() {
    if(this.isReady()==false) {
        console.log('ERROR! before' + this.constructor.name + 
                            '.adjust() call you needed to call this.switchToMe()!!');
    }
    this.ModelMat = popMatrix();
    pushMatrix(this.ModelMat);
}

VBObox5.prototype.draw = function() {
    if (runMode > 1){
        if (runMode == 2) runMode = 1;  // do one step
        // 1) DotFinder(): Find s0Dot from s0 & f0. 
        this.pSys.applyAllForces(this.pSys.S0, this.pSys.F0);
        this.pSys.dotMaker(this.pSys.S0dot, this.pSys.S0, g_timeStep);

        //2) Solver(): Find s1 from s0 & s0dot
        this.pSys.solver(g_timeStep, this.pSys.S0, this.pSys.S0dot, this.pSys.S1);

        // 3) Apply all constraints
        this.pSys.doConstraints(this.pSys.S1, this.pSys.S0, this.pSys.C0);
     
         // 4) Render
         this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);
        // gl.bufferSubData(gl.ARRAY_BUFFER, 0, this.pSys.S0, gl.DYNAMIC_DRAW);
        // gl.bufferSubData(gl.ARRAY_BUFFER, this.pSys.S0.length*this.FSIZE, this.pSys.S1, gl.DYNAMIC_DRAW);
        // gl.uniformMatrix4fv(this.u_ModelMatLoc, false, this.ModelMat.elements);
        // gl.drawArrays(gl.LINES, 0, this.partCount*2);

        // 5) Swap
        [this.pSys.S0, this.pSys.S1] = this.pSys.stateVecSwap(this.pSys.S0, this.pSys.S1);

    }
    else{  // paused. Only draw current state
        this.pSys.drawMe(this.pSys.S0, this.ModelMat, this.u_ModelMatLoc);
    }
}

VBObox5.prototype.isReady = function() {
    var isOK = true;

    if(gl.getParameter(gl.CURRENT_PROGRAM) != this.shaderLoc)  {
        console.log(this.constructor.name + 
                            '.isReady() false: shader program at this.shaderLoc not in use!');
        isOK = false;
    }
    if(gl.getParameter(gl.ARRAY_BUFFER_BINDING) != this.vboLoc) {
        console.log(this.constructor.name + 
                            '.isReady() false: vbo at this.vboLoc not in use!');
        isOK = false;
    }
    return isOK;
}