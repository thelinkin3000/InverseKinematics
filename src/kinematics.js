console.log('kinematics.js loaded');

function KinematicChain(joints) {
	var self = this;

	//a, d, alpha y theta son arrays que corresponden a los parámetros DH desde la base hasta la herramienta
	this.a = [];
	this.d = [];
	this.alpha = [];
	this.theta = [];

	//base_transform determina dónde se encuentra la base del robot.
	//j_transforms son los resultados de aplicar las matrices de traslación y rotación DH para cada articulación.
	this.base_transform = new THREE.Matrix4();
	this.j_transforms;

	this.j_mesh = [];
	this.l_mesh = [];

	//Colores para distinguir las partes del robot
	var l_colours = [0xff2121, 0xff96b21, 0xf7ff21, 0x4dff21, 0x2181ff, 0xb121ff],
		j_colour = 0xb2b2b2;

	//Dado el estado de cada cuarteto de parámetros a, d, theta, alpha, aplicar las matrices DH
	//para determinar la posición de cada articulación y de la herramienta.
	this.forward = function() {
		// Matriz de matrices
		this.j_transforms = [];
		// Variable que mantiene la matriz sobre la que se aplicará la transformación
		var transform = this.base_transform.clone();
		for (var i = 0; i < joints.length; i++) {
			console.log(`Articulacion ${i}: [d=${this.d[i]}, a=${this.a[i]}, alpha=${this.alpha[i]}, theta=${this.theta[i]}]`);
			// Aplicar la transformación i a la última articulación que calculamos.
			this.j_transforms[i] = transform.multiply(this.get_dh_matrix(i)).clone();
			this.j_mesh[i].matrix = this.j_transforms[i].clone();
			this.l_mesh[i].matrix = this.j_transforms[i].clone();
		}
	}

	//Dado un vector target, calcular el jacobiano inverso y obtener así los ángulos que aplicar
	//al estado actual para llegar al estado deseado.
	this.iterateIK = function(target) {
		var joint_centre, tip, to_tip, z_axis,
			jacobian_t = [], angles;

		//Computar la matriz jacobiana transpuesta
		for (var i = 0; i < this.theta.length; i++) {
			//Representa el punto en el espacio donde se encuentra la articulación
			joint_centre = new THREE.Vector3(...this.j_transforms[i].elements.slice(12, 15));

			//Representa el punto en el espacio donde se encuentra la herramienta
			tip = new THREE.Vector3(...this.j_transforms[this.theta.length-1].elements.slice(12, 15));
			
			//Resta el target del punto de la articulacion
			to_tip = tip.clone().sub(joint_centre);
			
			//Normaliza el vector de la articulación, y realiza un producto cruz con el vector resta para el jacobiano
			z_axis = new THREE.Vector3(...this.j_transforms[i].elements.slice(8, 11)).normalize();
			mv = z_axis.clone().cross(to_tip);

			//El jacobiano queda conformado, para esta articulación,
			//como el resultado del producto cruz entre el vector resta y el eje z de la articulación, y el eje z de la articulación
			jacobian_t.push([mv.x, mv.y, mv.z, z_axis.x, z_axis.y, z_axis.z]);
		}

		//delta_x(definido por sus ángulos) = J_transpose . tip_to_target_vector
		angles = dot(jacobian_t, target);

		//Determinar el factor de escalado para el método iterativo
		var alpha = 0.05;

		//Aplicar el factor a los ángulos calculados
		for (var i = 0; i < angles.length; i++) {
			angles[i] *= alpha;
		}

		return angles;
	}

	//Obtiene la matriz de transformación DH para la articulacion i.
	this.get_dh_matrix = function(index) {
		var am = a_matrix(this.a[index]),
			dm = d_matrix(this.d[index]),
			alm = alpha_matrix(this.alpha[index]),
			thm = theta_matrix(this.theta[index]),
			result = new THREE.Matrix4();

		result.multiply(alm);
		result.multiply(am);

		result.multiply(thm);
		result.multiply(dm);

		return result;
	}

	this.get_all_meshes = function() {
		return this.l_mesh.concat(this.j_mesh);
	}

	init();

	function init() {
		//populate dh parameter arrays. joints start from the base to the tip.
		for (var i = 0; i < joints.length; i++) {
			self.d.push(joints[i][0]);
			self.a.push(joints[i][1]);
			self.alpha.push(joints[i][2]);
			self.theta.push(joints[i][3]);
		}

		//populate mesh arrays
		for (var i = 0; i < joints.length; i++) {
			self.j_mesh[i] = make_j_mesh(i);
			self.l_mesh[i] = make_l_mesh(i);
		}

		//populate the transforms array for this joint angle config
		self.forward();
	}

	function dot(a, b) {
		var n, m, p;

		n = a.length;
		(a[0].constructor === Array) ? m = a[0].length : m = 1;
		(b[0].constructor === Array) ? p = b[0].length : p = 1;

		if (b[0].constructor === Array && b.length != m) {
			console.warn('KinematicChain>dot: matrix B is not a 2D matrix with cols equal to the rows of A');
			return;
		}

		var result;

		if (a[0].constructor !== Array && b[0].constructor !== Array) {
			//dot product of two 1D vectors
			result = 0;
			for (var i = 0; i < a.length; a++) {
				result += a[i] * b[i];
			}
			return result;
		} else if (a[0].constructor === Array && b[0].constructor !== Array) {
			//dot product a = 2D matrix with b = 1D vector
			result = [];
			for (var i = 0; i < b.length; i++) {
				result.push(0);
			}

			for (var i = 0; i < a.length; i++) {
				for (var j = 0; j < b.length; j++) {
					result[i] += b[j] * a[i][j];
				}
			}
			return result;
		} else {
			//dot product a and b are 2D matrices
			result = zeroes(n, p);

			for (var i = 0; i < n; i++) {
				for (var j = 0; j < p; j++) {
					for (var k = 0; k < m; k++) {
						result[i][j] += a[i][k] * b[k][j];
					}
				}
			}

			return result;
		}
	}

	function zeroes(rows, cols) {
		var array = [], row = [];
		for (var i = 0; i < rows; i++) {
			row.push(0);
		}
		for (var i = 0; i < cols; i++) {
			array.push(row.slice());
		}
		return array;
	}

	function make_j_mesh(index) {
		var j_geo = new THREE.CylinderGeometry(1, 1, 1);
		var j_mat = new THREE.MeshBasicMaterial( {color: j_colour, transparent:true, opacity:0.8} );

		//point the cylinder along the Z axis.
		j_geo.rotateX(Math.PI / 2);
		if (self.d[index] != 0) {
			//scale non-zero d values along the Z axis.
			j_geo.translate(0, 0, 0.5);
			j_geo.scale(1, 1, self.d[index]);
			j_geo.translate(0, 0, -self.d[index]);
		}

		var mesh = new THREE.Mesh(j_geo, j_mat);
		mesh.matrixAutoUpdate = false;

		if (index == 5) {
			mesh.add(new THREE.AxisHelper(10));	
		}

		return mesh;
	}

	function make_l_mesh(index) {
		var l_geo = new THREE.CylinderGeometry(1, 1, 1);
		var l_mat = new THREE.MeshBasicMaterial( {color: l_colours[index], transparent:true, opacity:0.8} );

		//point the cylinder along the X axis.
		l_geo.rotateZ(Math.PI / 2);
		if (index == 5) {
			l_geo.rotateY(Math.PI / 2);
			l_geo.scale(0.5, 0.5, 2);
			l_geo.translate(0, 0, 1);
		} else if (index+1 < self.theta.length && self.a[index+1] != 0) {
			//scale non-zero a values along the X axis.
			l_geo.translate(0.5, 0, 0);
			l_geo.scale(self.a[index+1], 1, 1);
		}

		var mesh = new THREE.Mesh(l_geo, l_mat);
		mesh.matrixAutoUpdate = false;

		return mesh;
	}

	//the d matrix pushes the joint along its Z axis.
	function d_matrix (val) {
		var m = new THREE.Matrix4();
		m.set(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, val,
			0, 0, 0, 1
		);
		return m;
	}

	//the a matrix pushes the next joint along this joint's
	//x axis.
	function a_matrix (val) {
		var m = new THREE.Matrix4();
		m.set(
			1, 0, 0, val,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1
		);
		return m;
	}

	//the alpha matrix refers to joint twist. it will rotate the
	//next joint in the chain along this joint's x axis.
	function alpha_matrix(val) {
		var s = Math.sin(radians(val));
		var c = Math.cos(radians(val));

		var m = new THREE.Matrix4();
		m.set(
			1, 0, 0, 0,
			0, c, -s, 0,
			0, s, c, 0,
			0, 0, 0, 1
		);
		return m;
	}

	//the theta matrix refers to the joint's angle. this is often
	//the parameter that is changed when moving the joint chain.
	function theta_matrix(val) {
		var s = Math.sin(radians(val));
		var c = Math.cos(radians(val));

		var m = new THREE.Matrix4();
		m.set(
			c, -s, 0, 0,
			s, c, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1
		);
		return m;
	}

	function radians (angle) {
		return (angle * Math.PI * 2 / 360);
	}
}