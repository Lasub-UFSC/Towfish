// frontend/src/main.js - Three.js GLTF Loader Example with npm

// Import necessary Three.js modules.
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
// Removed: import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

// Declare global variables
let scene, camera, renderer, model, pivot; // Removed: controls

/**
 * Initializes the Three.js scene and UI controls.
 */
function init() {
  const canvas = document.getElementById("threeJsCanvas");

  // Scene setup
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x01013F );

  // Camera setup
  camera = new THREE.PerspectiveCamera(
    75,
    window.innerWidth / window.innerHeight,
    0.1,
    1000
  );
  // Position the camera to a good viewing distance and point it at the center
  camera.position.set(0, 0, 5);
  camera.lookAt(0, 0, 0);

  // Renderer setup
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(window.devicePixelRatio);

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambientLight);
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(5, 5, 5).normalize();
  scene.add(directionalLight);

  // Removed: OrbitControls instantiation and setup

  // GLTFLoader
  const loader = new GLTFLoader();
  const modelUrl = "/3D/towfish.gltf";

  loader.load(
    modelUrl,
    function (gltf) {
      model = gltf.scene;

      // **New Code: Traverse the model and center the geometry**
      model.traverse((child) => {
        if (child.isMesh) {
          // Center the geometry of each mesh
          child.geometry.center();
          // Create a new material with the desired color
          const newMaterial = new THREE.MeshStandardMaterial({
            color: 0xff5c00,
            metalness: 0.5,
            roughness: 0.5,
          });

          // Assign the new material to the mesh
          child.material = newMaterial;
        }
      });

      // Set the model's position to the scene's center
      model.position.set(0, 0, 0);

      model.scale.set(0.02, 0.02, 0.02);

      model.position.set(0, 0, -10);

      model.rotation.x = 90 * (Math.PI / 180);
      model.rotation.y = 0 * (Math.PI / 180);
      model.rotation.z = -90 * (Math.PI / 180);
      const axesHelper = new THREE.AxesHelper( 1000 );
      model.add(axesHelper);
      scene.add(model);
      console.log("Model loaded successfully and centered:", gltf);
    },
    function (xhr) {
      console.log((xhr.loaded / xhr.total) * 100 + "% loaded");
    },
    function (error) {
      console.error("An error occurred loading the model:", error);
    }
  );

  window.addEventListener("resize", onWindowResize, false);

    // ws setup
    var ws = new WebSocket(`ws://localhost:8000/ws`);
    ws.onmessage = function(event) {
        let obj = JSON.parse(event.data)
        console.log(obj)
        updateModel(obj["Pitch"],obj["Roll"],0)
    };
}

function updateModel(rotationX,rotationY,rotationZ) {
  if (model) {
    model.rotation.x = parseFloat(90-rotationX) * (Math.PI / 180);
    model.rotation.y = parseFloat(-rotationY) * (Math.PI / 180);
    model.rotation.z = parseFloat(-90) * (Math.PI / 180);
  }
}

/**
 * Handles window resizing.
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
  renderer.render(scene, camera);
}

// Start everything when the window loads.
window.onload = function () {
  init();
  animate();
};
