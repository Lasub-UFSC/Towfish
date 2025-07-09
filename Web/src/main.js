// frontend/src/main.js - Three.js GLTF Loader Example with npm

// Import necessary Three.js modules.
// When using npm, you import directly from 'three' and its sub-paths.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

// Declare global variables for the scene, camera, renderer, and controls.
let scene, camera, renderer, controls;

/**
 * Initializes the Three.js scene, camera, renderer, lights, and loads a GLTF model.
 */
function init() {
    const canvas = document.getElementById('threeJsCanvas');

    // 1. Scene setup
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x333344);

    // 2. Camera setup
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 0.5, 2); // Position the camera to view the model better

    // 3. Renderer setup
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);

    // 4. Lights: GLTF models often rely on lights for their appearance.
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 5, 5).normalize();
    scene.add(directionalLight);

    // 5. OrbitControls: Allows users to pan, zoom, and rotate the camera with mouse/touch.
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 1;
    controls.maxDistance = 10;
    controls.target.set(0, 0.5, 0);
    controls.update();

    // 6. GLTFLoader: Instantiate the loader for .gltf and .glb models.
    const loader = new GLTFLoader();

    // Load a sample GLTF model.
    // To fix the 404, we will now load the model from a local path within your project.
    // You need to place the 'FlightHelmet' folder (containing FlightHelmet.gltf,
    // FlightHelmet.bin, and textures) inside your 'frontend/public/models/' directory.
    // The path here is relative to the 'public' folder root.
    const modelUrl = '/3D/towfish.gltf';

    loader.load(
        modelUrl,
        function (gltf) {
            scene.add(gltf.scene);
            console.log('Model loaded successfully:', gltf);
        },
        function (xhr) {
            console.log((xhr.loaded / xhr.total * 100) + '% loaded');
        },
        function (error) {
            console.error('An error occurred loading the model:', error);
        }
    );

    // Add event listener for window resizing
    window.addEventListener('resize', onWindowResize, false);
}

/**
 * Handles window resizing to make the Three.js canvas responsive.
 */
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

/**
 * The animation loop.
 */
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Ensure the Three.js initialization and animation loop start after the window has loaded.
window.onload = function () {
    init();
    animate();
};
