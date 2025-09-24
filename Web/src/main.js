// Import necessary Three.js modules.
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Chart from 'chart.js/auto';

// Declare global variables
let scene, camera, renderer, model, Xaxis, Yaxis, Zaxis, controls, chart;
const pitchText = document.getElementById("pitchText");
const rollText = document.getElementById("rollText");
const depthText = document.getElementById("depthText");
const innerDepthBar = document.getElementById("innerDepthBar");

let startTimestamp = 0;
/**
 * init3Dializes the Three.js scene and UI controls.
 */
function init3D() {
  const canvas = document.getElementById("threeJsCanvas");

  // Scene setup
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x01013F);

  // Camera setup
  camera = new THREE.PerspectiveCamera(
    75,
    window.innerWidth / window.innerHeight,
    0.1,
    1000
  );
  // Position the camera to a good viewing distance and point it at the center
  camera.position.set(-1, 1, 17);
  camera.lookAt(0, 0, 0);

  // Renderer setup
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(window.devicePixelRatio);

  controls = new OrbitControls(camera, renderer.domElement);

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambientLight);
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(5, 5, 5).normalize();
  scene.add(directionalLight);


  // GLTFLoader
  const loader = new GLTFLoader();
  const modelUrl = "3D/towfish.gltf";

  loader.load(
    modelUrl,
    function (gltf) {
      model = gltf.scene;
      const axesHelper = new THREE.AxesHelper(25);
      Xaxis = new THREE.Group();
      Yaxis = new THREE.Group();
      Zaxis = new THREE.Group();

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
      model.scale.set(0.025, 0.025, 0.025);

      model.position.set(0, 0, 0);

      Zaxis.add(model);
      Yaxis.add(Zaxis);
      Xaxis.add(Yaxis);

      axesHelper.rotation.y = -90 * (Math.PI / 180);
      Xaxis.rotation.x = 90 * (Math.PI / 180);
      Yaxis.rotation.y = 0 * (Math.PI / 180);
      Zaxis.rotation.z = -90 * (Math.PI / 180);

      scene.add(Xaxis);
      scene.add(axesHelper);
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

}

function updateModel(rotationX, rotationY, rotationZ, depth) {
  if (model) {
    Xaxis.rotation.x = parseFloat(90 - rotationX) * (Math.PI / 180);
    Yaxis.rotation.y = parseFloat(-rotationY) * (Math.PI / 180);
    Zaxis.rotation.z = parseFloat(rotationZ) * (Math.PI / 180);
  }
  pitchText.innerText = rotationY.toFixed(2);
  rollText.innerText = rotationX.toFixed(2);
  depthText.innerText = depth.toFixed(2);
  innerDepthBar.style.height = depth + "%";
}

function updateChart(pitch, timestamp) {
  if (startTimestamp == 0) {
    startTimestamp = timestamp;
  }
  // Add the new data point to the dataset
  chart.data.labels.push((timestamp - startTimestamp).toFixed(2)); // Add new label
  chart.data.datasets[0].data.push(pitch); // Add new data point

  // Limit the number of data points to keep the chart from getting too crowded
  const maxDataPoints = 200 ;
  if (chart.data.datasets[0].data.length > maxDataPoints) {
    chart.data.labels.shift(timestamp); // Add new label
    chart.data.datasets[0].data.shift();
  }
  chart.update();
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
  controls.update();
  renderer.render(scene, camera);
}

function initChart() {
  const ctx = document.getElementById('myLineChart').getContext('2d');

  // Sample data for the line chart
  const initialData = {
    labels: [],
    datasets: [{
      data: [],
      fill: false,
      borderColor: 'rgb(75, 192, 192)',
      tension: 0.1
    }]
  };

  // Configuration options for the chart
  const config = {
    type: 'line',
    data: initialData,
    options: {
      elements: {
        point: {
          pointStyle: 'line' // or 'rect', 'star', etc.
        }
      },
      animation: {
        duration: 0,
      },
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: {
          display: false // This line hides the legend
        },
        tooltip: {
          mode: 'index',
          intersect: false,
        }
      },
      scales: {
        x: {
          // Set the scale type to 'time' to handle timestamps
          time: {
            unit: 'second',
            displayFormats: {
              second: 'HH:mm:ss'
            }
          },
          title: {
            display: true,
            text: 'Tempo (s)',
            color: "#FFF",
            font: {
              family: 'Arial',
            }
          }
        },
        y: {
          suggestedMin:-5,
          suggestedMax:5,
          title: {
            display: true,
            text: 'Pitch ( °)',
            color: "#FFF",
            font: {
              family: 'Arial',
            }
          },
          ticks: {
            color: "#FFF",
            font: {
              family: 'Arial',
            }
          },
          grid: {
            color: "#CCCCCC",
          }
        }
      }
    }
  };

  // Create the new chart instance
  chart = new Chart(ctx, config);
};

// Start everything when the window loads.
window.onload = function () {
  init3D();
  initChart();
  // ws setup
  var ws = new WebSocket(`ws://localhost:8000/ws`);
  ws.onmessage = function (event) {
    let obj = JSON.parse(event.data)
    updateModel(obj["Pitch"], obj["Roll"], -90, obj["Depth"])
    updateChart(obj["Roll"], obj["Timestamp"])
  };

  animate();
};
