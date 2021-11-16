Ammo().then(function(Ammo) {

    // Detects webgl
    if ( ! Detector.webgl ) {
        Detector.addGetWebGLMessage();
        document.getElementById( 'container' ).innerHTML = "";
    }

    // - Global variables -

    // Graphics variables
    var container;
    var camera, controls, scene, renderer;
    var textureLoader;
    var clock = new THREE.Clock();

    // Physics variables
    var gravityConstant = -9.8;
    var collisionConfiguration;
    var dispatcher;
    var broadphase;
    var solver;
    var physicsWorld;
    var rigidBodies = [];
    var margin = 0.05;
    var hinge;
    var rope;
    var transformAux1 = new Ammo.btTransform();

    var armMovement = 0;

    // - Main code -

    init();
    animate();


    // - Functions -

    function init() {
        initGraphics();
        initPhysics();
        createObjects();
        initInput();
    }

    function initGraphics() {

        container = document.getElementById( 'container' );

        camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 0.2, 2000 );

        scene = new THREE.Scene();

        camera.position.x = -7;
        camera.position.y = 5;
        camera.position.z =  8;

        controls = new THREE.OrbitControls( camera );
        controls.target.y = 2;

        renderer = new THREE.WebGLRenderer();
        renderer.setClearColor( 0xbfd1e5 );
        renderer.setPixelRatio( window.devicePixelRatio );
        renderer.setSize( window.innerWidth, window.innerHeight );
        renderer.shadowMap.enabled = true;

        textureLoader = new THREE.TextureLoader();

        var ambientLight = new THREE.AmbientLight( 0x404040 );
        scene.add( ambientLight );

        var light = new THREE.DirectionalLight( 0xffffff, 1 );
        light.position.set( -10, 10, 5 );
        light.castShadow = true;
        var d = 10;
        light.shadow.camera.left = -d;
        light.shadow.camera.right = d;
        light.shadow.camera.top = d;
        light.shadow.camera.bottom = -d;

        light.shadow.camera.near = 2;
        light.shadow.camera.far = 50;

        light.shadow.mapSize.x = 1024;
        light.shadow.mapSize.y = 1024;

        scene.add( light );


        container.innerHTML = "";

        container.appendChild( renderer.domElement );

        window.addEventListener( 'resize', onWindowResize, false );

    }

    function initPhysics() {

        // Physics configuration

        collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
        dispatcher = new Ammo.btCollisionDispatcher( collisionConfiguration );
        broadphase = new Ammo.btDbvtBroadphase();
        solver = new Ammo.btSequentialImpulseConstraintSolver();
        softBodySolver = new Ammo.btDefaultSoftBodySolver();
        physicsWorld = new Ammo.btSoftRigidDynamicsWorld( dispatcher, broadphase, solver, softBodySolver);
        physicsWorld.setGravity( new Ammo.btVector3( 0, gravityConstant, 0 ) );
        physicsWorld.getWorldInfo().set_m_gravity( new Ammo.btVector3( 0, gravityConstant, 0 ) );
    }

    function createObjects() {

        var pos = new THREE.Vector3();
        var quat = new THREE.Quaternion();

        // Ground
        pos.set( 0, - 0.5, 0 );
        quat.set( 0, 0, 0, 1 );
        var ground = createParalellepiped( 40, 1, 40, 0, pos, quat, new THREE.MeshPhongMaterial( { color: 0xFFFFFF } ) );
        ground.castShadow = true;
        ground.receiveShadow = true;
        textureLoader.load( "./assets/texture/grid.png", function( texture ) {
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
            texture.repeat.set( 40, 40 );
            ground.material.map = texture;
            ground.material.needsUpdate = true;
        } );
        
        // The rope
        // Rope graphic object
        var ropeNumSegments = 10;
        var ropeLength = 4;
        var ropeMass = 3;
        var ropePos = new THREE.Vector3(-3);

        var segmentLength = ropeLength / ropeNumSegments;
        var ropeGeometry = new THREE.BufferGeometry();
        var ropeMaterial = new THREE.LineBasicMaterial( { color: "#FF0000" } );
        var ropePositions = [];
        var ropeIndices = [];

        for ( var i = 0; i < ropeNumSegments + 1; i++ ) {
            ropePositions.push( ropePos.x, ropePos.y + i * segmentLength, ropePos.z );
        }

        for ( var i = 0; i < ropeNumSegments; i++ ) {
            ropeIndices.push( i, i + 1 );
        }

        ropeGeometry.setIndex( new THREE.BufferAttribute( new Uint16Array( ropeIndices ), 1 ) );
        ropeGeometry.addAttribute( 'position', new THREE.BufferAttribute( new Float32Array( ropePositions ), 3 ) );
        ropeGeometry.computeBoundingSphere();
        rope = new THREE.LineSegments( ropeGeometry, ropeMaterial );
        rope.castShadow = true;
        rope.receiveShadow = true;
        scene.add( rope );

        // Rope physic object
        var softBodyHelpers = new Ammo.btSoftBodyHelpers();
        var ropeStart = new Ammo.btVector3( ropePos.x, ropePos.y, ropePos.z );
        var ropeEnd = new Ammo.btVector3( ropePos.x, ropePos.y + ropeLength, ropePos.z );
        var ropeSoftBody = softBodyHelpers.CreateRope( physicsWorld.getWorldInfo(), ropeStart, ropeEnd, ropeNumSegments - 1);
        var sbConfig = ropeSoftBody.get_m_cfg();
        sbConfig.set_viterations( 10 );
        sbConfig.set_piterations( 10 );
        ropeSoftBody.setTotalMass( ropeMass, false )
        Ammo.castObject( ropeSoftBody, Ammo.btCollisionObject ).getCollisionShape().setMargin( margin * 3 );
        physicsWorld.addSoftBody( ropeSoftBody, 1, -1 );
        rope.userData.physicsBody = ropeSoftBody;

        // The base
        var armMass = 2;
        var armLength = 3;
        var pylonHeight = 4;
        var baseMaterial = new THREE.MeshPhongMaterial( { color: 0x606060 } );
        pos.set( ropePos.x, 0.1, ropePos.z - armLength );
        quat.set( 0, 0, 0, 1 );
        var base = createParalellepiped( 1, 0.2, 1, 0, pos, quat, baseMaterial );
        base.castShadow = true;
        base.receiveShadow = true;
        pos.set( ropePos.x, 0.5 * pylonHeight, ropePos.z - armLength );
        var pylon = createParalellepiped( 0.4, pylonHeight, 0.4, 0, pos, quat, baseMaterial );
        pylon.castShadow = true;
        pylon.receiveShadow = true;
        pos.set( ropePos.x, pylonHeight + 0.2, ropePos.z - 0.5 * armLength );
        var arm = createParalellepiped( 0.4, 0.4, armLength + 0.4, armMass, pos, quat, baseMaterial );
        arm.castShadow = true;
        arm.receiveShadow = true;

        // Glue the rope extremes to the ball and the arm
        var influence = 1;
        ropeSoftBody.appendAnchor( ropeNumSegments, arm.userData.physicsBody, true, influence );

        // Hinge constraint to move the arm
        var pivotA = new Ammo.btVector3( 0, pylonHeight * 0.5, 0 );
        var pivotB = new Ammo.btVector3( 0, -0.2, - armLength * 0.5 );
        var axis = new Ammo.btVector3( 0, 1, 0 );
        hinge = new Ammo.btHingeConstraint( pylon.userData.physicsBody, arm.userData.physicsBody, pivotA, pivotB, axis, axis, true );
        physicsWorld.addConstraint( hinge, true );
    }

    function createParalellepiped( sx, sy, sz, mass, pos, quat, material ) {

        var threeObject = new THREE.Mesh( new THREE.BoxGeometry( sx, sy, sz, 1, 1, 1 ), material );
        var shape = new Ammo.btBoxShape( new Ammo.btVector3( sx * 0.5, sy * 0.5, sz * 0.5 ) );
        shape.setMargin( margin );

        createRigidBody( threeObject, shape, mass, pos, quat );

        return threeObject;

    }

    function createRigidBody( threeObject, physicsShape, mass, pos, quat ) {

        threeObject.position.copy( pos );
        threeObject.quaternion.copy( quat );

        var transform = new Ammo.btTransform();
        transform.setIdentity();
        transform.setOrigin( new Ammo.btVector3( pos.x, pos.y, pos.z ) );
        transform.setRotation( new Ammo.btQuaternion( quat.x, quat.y, quat.z, quat.w ) );
        var motionState = new Ammo.btDefaultMotionState( transform );

        var localInertia = new Ammo.btVector3( 0, 0, 0 );
        physicsShape.calculateLocalInertia( mass, localInertia );

        var rbInfo = new Ammo.btRigidBodyConstructionInfo( mass, motionState, physicsShape, localInertia );
        var body = new Ammo.btRigidBody( rbInfo );

        threeObject.userData.physicsBody = body;

        scene.add( threeObject );

        if ( mass > 0 ) {
            rigidBodies.push( threeObject );

            // Disable deactivation
            body.setActivationState( 4 );
        }

        physicsWorld.addRigidBody( body );

    }

    function initInput() {

        window.addEventListener( 'keydown', function( event ) {

            switch ( event.keyCode ) {
                // D
                case 68:
                    armMovement = 1;
                break;

                // A
                case 65:
                    armMovement = - 1;
                break;
            }

        }, false );

        window.addEventListener( 'keyup', function( event ) {

            armMovement = 0;

        }, false );

    }

    function onWindowResize() {

        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();

        renderer.setSize( window.innerWidth, window.innerHeight );

    }

    function animate() {
        requestAnimationFrame( animate );
        render();
    }

    function render() {

        var deltaTime = clock.getDelta();

        updatePhysics( deltaTime );

        controls.update( deltaTime );

        renderer.render( scene, camera );

        time += deltaTime;

    }

    function updatePhysics( deltaTime ) {

        // Hinge control
        hinge.enableAngularMotor( true, 1.5 * armMovement, 50 );

        // Step world
        physicsWorld.stepSimulation( deltaTime, 10 );

        // Update rope
        var softBody = rope.userData.physicsBody;
        var ropePositions = rope.geometry.attributes.position.array;
        var numVerts = ropePositions.length / 3;
        var nodes = softBody.get_m_nodes();
        var indexFloat = 0;
        for ( var i = 0; i < numVerts; i ++ ) {

            var node = nodes.at( i );
            var nodePos = node.get_m_x();
            ropePositions[ indexFloat++ ] = nodePos.x();
            ropePositions[ indexFloat++ ] = nodePos.y();
            ropePositions[ indexFloat++ ] = nodePos.z();

        }
        rope.geometry.attributes.position.needsUpdate = true;

        // Update rigid bodies
        for ( var i = 0, il = rigidBodies.length; i < il; i++ ) {
            var objThree = rigidBodies[ i ];
            var objPhys = objThree.userData.physicsBody;
            var ms = objPhys.getMotionState();
            if ( ms ) {

                ms.getWorldTransform( transformAux1 );
                var p = transformAux1.getOrigin();
                var q = transformAux1.getRotation();
                objThree.position.set( p.x(), p.y(), p.z() );
                objThree.quaternion.set( q.x(), q.y(), q.z(), q.w() );

              }
        }

    }

});