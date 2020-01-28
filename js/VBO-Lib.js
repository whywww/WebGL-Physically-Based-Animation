var floatsPerVertex = 7;

/****************** VBObox1 ***********************/
// Rigid Objects
function VBObox1(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec3 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = vec4(a_Color, 1.0); \n' +	
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

    var totalSize = gndVerts.length + axisVerts.length 
                    + cubeVerts.length;
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

    this.vboContents = vertices;
    this.vboVerts = vcount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos =  4;
    this.vboFcount_a_Colr = 3;
    
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
    'attribute vec3 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = vec4(a_Color, 1.0); \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    'varying vec4 v_Color; \n' +
    'void main() {\n' +
    '		float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '		if(dist < 0.5) { \n' +	
    '			gl_FragColor = vec4((1.0-2.0*dist)*v_Color.rgb, 1.0);\n' +
    '    	}\n' +
    '		else { \n' + 
    '			discard;\n' +
    '		}\n' +
    '}\n';
    // initiate particle system
    pSys = new CPartSys();
    var partCount = 100;
    var forces = [F_GRAV_E, F_DRAG];
    var walls = [WTYPE_YWALL_LO, WTYPE_YWALL_HI, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    pSys.init(partCount, forces, walls);

    this.vboContents = pSys.S0;
    this.vboVerts = pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos =  4;
    this.vboFcount_a_Colr = 3;

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
}

VBObox2.prototype.draw = function() {
    this.ModelMat = popMatrix();
    drawBall(this.ModelMat, this.u_ModelMatLoc);
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
// Spring
function VBObox3(){
    this.VERT_SRC = 
    'precision mediump float;\n' +

    'uniform mat4 u_ModelMatrix;\n' +

    'attribute vec4 a_Position;\n' +
    'attribute vec3 a_Color;\n' +

    'varying vec4 v_Color; \n' +

    'void main() {\n' +
    '   gl_PointSize = 20.0;\n' + 
    '   gl_Position = u_ModelMatrix * a_Position; \n' +
    '	v_Color = vec4(a_Color, 1.0); \n' +	
    '} \n';

    this.FRAG_SRC = 
    'precision mediump float;\n' +
    'varying vec4 v_Color; \n' +
    'void main() {\n' +
    '		float dist = distance(gl_PointCoord, vec2(0.5, 0.5)); \n' +
    '		if(dist < 0.5) { \n' +	
    '			gl_FragColor = vec4((1.0-2.0*dist)*v_Color.rgb, 1.0);\n' +
    '    	}\n' +
    '		else { \n' + 
    '			discard;\n' +
    '		}\n' +
    '}\n';
    // initiate particle system
    pSys = new CPartSys();
    var partCount = 100;
    var forces = [F_GRAV_E, F_DRAG];
    var walls = [WTYPE_YWALL_LO, WTYPE_YWALL_HI, WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_ZWALL_LO, WTYPE_ZWALL_HI];
    pSys.init(partCount, forces, walls);

    this.vboContents = pSys.S0;
    this.vboVerts = pSys.partCount;
	this.FSIZE = this.vboContents.BYTES_PER_ELEMENT;
    this.vboBytes = this.vboContents.length * this.FSIZE;
    this.vboStride = this.vboBytes / this.vboVerts; 
    
    this.vboFcount_a_Pos =  4;
    this.vboFcount_a_Colr = 3;

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
}

VBObox3.prototype.draw = function() {
    this.ModelMat = popMatrix();
    drawBall(this.ModelMat, this.u_ModelMatLoc);
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