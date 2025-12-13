import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import URDFLoader from 'urdf-loader';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 2, 5);

const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 5);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.near = 0.5;
directionalLight.shadow.camera.far = 50;
directionalLight.shadow.camera.left = -10;
directionalLight.shadow.camera.right = 10;
directionalLight.shadow.camera.top = 10;
directionalLight.shadow.camera.bottom = -10;
scene.add(directionalLight);

const fillLight = new THREE.DirectionalLight(0xffffff, 0.3);
fillLight.position.set(-5, 5, -5);
scene.add(fillLight);

const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
scene.add(gridHelper);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.minDistance = 1;
controls.maxDistance = 20;
controls.target.set(0, 1, 0);

let robot = null;
let robotInitialPosition = new THREE.Vector3();
let allJoints = new Map();
let jointInitialValues = new Map();

let calfLinks = [];
let calfLinkInitialRotations = new Map();
let hipLinks = [];
let hipLinkInitialRotations = new Map();
let shoulderLinks = [];
let shoulderLinkInitialRotations = new Map();
let animationCycle = 0;
const ANIMATION_DURATION = 0.4;
const clock = new THREE.Clock();

let legJoints = {
    frontLeft: { all: [] },
    frontRight: { all: [] },
    backLeft: { all: [] },
    backRight: { all: [] }
};

const loadingManager = new THREE.LoadingManager();
if (!loadingManager.resolveURL) {
    loadingManager.resolveURL = (url) => url;
}

const loader = new URDFLoader(loadingManager);

const originalConsoleError = console.error;
console.error = function(...args) {
    const errorMsg = args[0]?.toString() || '';
    if (errorMsg.includes('ColladaLoader') || errorMsg.includes('Failed to parse collada')) {
        return;
    }
    if (errorMsg.includes("Cannot read properties of null") && errorMsg.includes("scene")) {
        return;
    }
    originalConsoleError.apply(console, args);
};

loader.load(
    './Assets/b2_description/urdf/b2_description.urdf',
    (urdfRobot) => {
        robot = urdfRobot;
        
        scene.add(robot);
        findAndOrganizeJoints(robot);
        
        setTimeout(() => {
            robot.traverse((child) => {
                if (child.isMesh) {
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            });
            
            const box = new THREE.Box3().setFromObject(robot);
            const center = box.getCenter(new THREE.Vector3());
            const size = box.getSize(new THREE.Vector3());
            
            robot.rotation.set(-Math.PI / 2, 0, 0);
            
            if (size.x === 0 && size.y === 0 && size.z === 0) {
                robot.position.set(0, 0.5, 0);
                robotInitialPosition.copy(robot.position);
                camera.position.set(0, 2, 5);
                controls.target.set(0, 0.5, 0);
            } else {
                robot.position.sub(center);
                robotInitialPosition.copy(robot.position);
                
                const maxDim = Math.max(size.x, size.y, size.z);
                if (maxDim > 3) {
                    const scale = 3 / maxDim;
                    robot.scale.multiplyScalar(scale);
                }
                
                const finalBox = new THREE.Box3().setFromObject(robot);
                const finalSize = finalBox.getSize(new THREE.Vector3());
                const finalCenter = finalBox.getCenter(new THREE.Vector3());
                const finalMaxDim = Math.max(finalSize.x, finalSize.y, finalSize.z);
                
                camera.position.set(finalCenter.x, finalCenter.y + finalMaxDim * 0.5, finalCenter.z + finalMaxDim * 1.5);
                controls.target.copy(finalCenter);
            }
            
            setJointsToStandingPose();
            document.getElementById('loading').style.display = 'none';
            
            console.log('Robot loaded. Total joints:', allJoints.size);
            console.log('Leg joints found:', {
                FL: legJoints.frontLeft.all,
                FR: legJoints.frontRight.all,
                BL: legJoints.backLeft.all,
                BR: legJoints.backRight.all
            });
            
            if (allJoints.size > 0) {
                const firstJoint = Array.from(allJoints.values())[0];
                console.log('Sample joint methods:', {
                    hasSetValue: typeof firstJoint.setValue === 'function',
                    hasSetJointValue: typeof firstJoint.setJointValue === 'function',
                    hasJointValue: firstJoint.jointValue !== undefined,
                    jointType: firstJoint.constructor.name
                });
            }
        }, 500);
    },
    {
        packages: {
            'b2_description': './Assets/b2_description'
        },
        workingPath: './Assets/b2_description/urdf/',
        loadMeshCb: (path, manager, done) => {
            let meshPath = path;
            
            if (path.includes('package://b2_description/')) {
                meshPath = path.replace('package://b2_description/', './Assets/b2_description/');
            } else if (path.startsWith('/Assets/')) {
                meshPath = '.' + path;
            } else if (!path.startsWith('./') && !path.startsWith('/') && !path.startsWith('http')) {
                if (path.includes('meshes/')) {
                    meshPath = path.startsWith('./') ? path : './' + path;
                } else {
                    const fileName = path.split('/').pop();
                    meshPath = './Assets/b2_description/meshes/' + fileName;
                }
            }
            
            if (/\.dae$/i.test(meshPath)) {
                const loader = new ColladaLoader(manager);
                loader.load(meshPath, (dae) => {
                    done(dae && dae.scene ? dae.scene : null);
                }, undefined, () => done(null));
            } else if (/\.stl$/i.test(meshPath)) {
                const loader = new STLLoader(manager);
                loader.load(meshPath, (geom) => {
                    const mesh = new THREE.Mesh(geom, new THREE.MeshPhongMaterial());
                    done(mesh);
                }, undefined, () => done(null));
            } else {
                done(null);
            }
        }
    }
);

function findAndOrganizeJoints(urdfRobot) {
    if (!urdfRobot.joints) return;
    
    Object.keys(urdfRobot.joints).forEach((jointName) => {
        const joint = urdfRobot.joints[jointName];
        allJoints.set(jointName, joint);
        const initialValue = joint.jointValue !== undefined ? joint.jointValue : 0;
        jointInitialValues.set(jointName, initialValue);
    });
    
    const allJointNames = Array.from(allJoints.keys());
    console.log('All joint names:', allJointNames);
    
    Array.from(allJoints.keys()).forEach((jointName) => {
        const nameLower = jointName.toLowerCase();
        
        if (nameLower.includes('fl_') || (nameLower.includes('front') && nameLower.includes('left'))) {
            if (nameLower.includes('hip') || nameLower.includes('thigh') || nameLower.includes('calf') || 
                nameLower.includes('shin') || nameLower.includes('knee') || nameLower.includes('ankle') ||
                nameLower.includes('foot') || nameLower.includes('rotor')) {
                legJoints.frontLeft.all.push(jointName);
            }
        } else if (nameLower.includes('fr_') || (nameLower.includes('front') && nameLower.includes('right'))) {
            if (nameLower.includes('hip') || nameLower.includes('thigh') || nameLower.includes('calf') || 
                nameLower.includes('shin') || nameLower.includes('knee') || nameLower.includes('ankle') ||
                nameLower.includes('foot') || nameLower.includes('rotor')) {
                legJoints.frontRight.all.push(jointName);
            }
        } else if (nameLower.includes('hl_') || nameLower.includes('rl_') || nameLower.includes('hind_left') || 
                 (nameLower.includes('back') && nameLower.includes('left')) ||
                 (nameLower.includes('hind') && nameLower.includes('left')) ||
                 (nameLower.includes('rear') && nameLower.includes('left'))) {
            if (nameLower.includes('hip') || nameLower.includes('thigh') || nameLower.includes('calf') || 
                nameLower.includes('shin') || nameLower.includes('knee') || nameLower.includes('ankle') ||
                nameLower.includes('foot') || nameLower.includes('rotor')) {
                legJoints.backLeft.all.push(jointName);
            }
        } else if (nameLower.includes('hr_') || nameLower.includes('rr_') || nameLower.includes('hind_right') ||
                 (nameLower.includes('back') && nameLower.includes('right')) ||
                 (nameLower.includes('hind') && nameLower.includes('right')) ||
                 (nameLower.includes('rear') && nameLower.includes('right'))) {
            if (nameLower.includes('hip') || nameLower.includes('thigh') || nameLower.includes('calf') || 
                nameLower.includes('shin') || nameLower.includes('knee') || nameLower.includes('ankle') ||
                nameLower.includes('foot') || nameLower.includes('rotor')) {
                legJoints.backRight.all.push(jointName);
            }
        }
    });
    
    console.log('Leg joints organized:', {
        FL: legJoints.frontLeft.all,
        FR: legJoints.frontRight.all,
        BL: legJoints.backLeft.all,
        BR: legJoints.backRight.all
    });
}

function setJointsToStandingPose() {
    if (allJoints.size === 0) return;
    
    const angle30Deg = Math.PI / 6;
    const angle45Deg = Math.PI / 4;
    const standingHip = 0.0;
    const standingHipY = 0.0;
    const standingHipX = 0.0;
    const standingThigh = 0.0;
    const standingCalf = -Math.PI / 2;
    
    const thighLinks = [];
    calfLinks = [];
    calfLinkInitialRotations.clear();
    hipLinks = [];
    hipLinkInitialRotations.clear();
    shoulderLinks = [];
    shoulderLinkInitialRotations.clear();
    
    const setJointToStanding = (jointName, value) => {
        if (!jointName) return;
        const joint = allJoints.get(jointName);
        if (joint) {
            const nameLower = jointName.toLowerCase();
            const oldValue = joint.jointValue;
            
            if (joint.jointValue !== undefined) {
                joint.jointValue = value;
            }
            if (typeof joint.setValue === 'function') {
                joint.setValue(value);
            } else if (typeof joint.setJointValue === 'function') {
                joint.setJointValue(value);
            }
            if (joint.update && typeof joint.update === 'function') {
                joint.update();
            }
            if (joint.link) {
                if (joint.link.updateMatrixWorld) {
                    joint.link.updateMatrixWorld(true);
                }
                if (joint.link.traverse) {
                    joint.link.traverse((child) => {
                        if (child.updateMatrixWorld) {
                            child.updateMatrixWorld(false);
                        }
                    });
                }
            }
            if (joint.parent) {
                if (joint.parent.updateMatrixWorld) {
                    joint.parent.updateMatrixWorld(true);
                }
            }
            jointInitialValues.set(jointName, value);
            
            if (nameLower.includes('calf')) {
                console.log(`Calf joint ${jointName}: set to ${value.toFixed(3)} (${(value * 180 / Math.PI).toFixed(1)}°), actual: ${joint.jointValue?.toFixed(3) || 'undefined'}`);
            }
        }
    };
    
    Object.values(legJoints).forEach(leg => {
        leg.all.forEach(jointName => {
            const nameLower = jointName.toLowerCase();
            if (nameLower.includes('rotor')) {
                return;
            }
            if (nameLower.includes('hip_joint')) {
                setJointToStanding(jointName, standingHip);
                const joint = allJoints.get(jointName);
                if (joint && joint.link) {
                    hipLinks.push(joint.link);
                    const initialRotation = joint.link.rotation.y;
                    hipLinkInitialRotations.set(joint.link, initialRotation);
                }
            } else if (nameLower.includes('thigh_joint')) {
                setJointToStanding(jointName, standingThigh);
                const joint = allJoints.get(jointName);
                if (joint && joint.link) {
                    thighLinks.push(joint.link);
                }
            } else if (nameLower.includes('calf_joint')) {
                console.log(`Setting calf joint ${jointName} to ${standingCalf} radians (${(standingCalf * 180 / Math.PI).toFixed(1)} degrees)`);
                setJointToStanding(jointName, standingCalf);
                const joint = allJoints.get(jointName);
                if (joint && joint.link) {
                    calfLinks.push(joint.link);
                    console.log(`  Found calf link: ${joint.link.name || 'unnamed'}`);
                }
            } else if (nameLower.includes('foot_joint')) {
                setJointToStanding(jointName, 0);
            } else {
                setJointToStanding(jointName, 0);
            }
        });
    });
    
    thighLinks.forEach(link => {
        if (link) {
            console.log(`Rotating thigh link ${link.name || 'unnamed'} by ${(angle45Deg * 180 / Math.PI).toFixed(1)} degrees`);
            link.rotation.y += angle30Deg;
            link.updateMatrixWorld(true);
        }
    });
    
    calfLinks.forEach(link => {
        if (link) {
            const initialRotation = link.rotation.y;
            calfLinkInitialRotations.set(link, initialRotation);
            console.log(`Setting initial calf link rotation for ${link.name || 'unnamed'}: ${(initialRotation * 180 / Math.PI).toFixed(1)} degrees`);
            link.rotation.y -= angle30Deg * 2;
            link.updateMatrixWorld(true);
        }
    });
    
    if (robot) {
        const foundThighLinks = [];
        const foundCalfLinks = [];
        robot.traverse((child) => {
            const childName = (child.name || '').toLowerCase();
            if (childName.includes('thigh') && !childName.includes('joint') && !childName.includes('rotor')) {
                foundThighLinks.push(child);
                console.log(`Found thigh link by name: ${child.name}`);
            }
            if (childName.includes('calf') && !childName.includes('joint') && !childName.includes('rotor')) {
                foundCalfLinks.push(child);
                console.log(`Found calf link by name: ${child.name}`);
            }
            if (child.updateMatrixWorld) {
                child.updateMatrixWorld(false);
            }
        });
        
        foundThighLinks.forEach(link => {
            console.log(`Rotating found thigh link ${link.name} by ${(angle45Deg * 180 / Math.PI).toFixed(1)} degrees`);
            link.rotation.y += angle30Deg;
            link.updateMatrixWorld(true);
        });
        
        foundCalfLinks.forEach(link => {
            if (!calfLinks.includes(link)) {
                calfLinks.push(link);
                const initialRotation = link.rotation.y;
                calfLinkInitialRotations.set(link, initialRotation);
                console.log(`Setting initial calf link rotation for found link ${link.name}: ${(initialRotation * 180 / Math.PI).toFixed(1)} degrees`);
            }
            link.rotation.y -= angle30Deg * 2;
            link.updateMatrixWorld(true);
        });
        
        const foundHipLinks = [];
        const foundShoulderLinks = [];
        robot.traverse((child) => {
            const childName = (child.name || '').toLowerCase();
            if ((childName.includes('hip') || childName.includes('shoulder')) && 
                !childName.includes('joint') && !childName.includes('rotor') && 
                !childName.includes('thigh') && !childName.includes('calf')) {
                if (childName.includes('shoulder')) {
                    foundShoulderLinks.push(child);
                } else if (childName.includes('hip')) {
                    foundHipLinks.push(child);
                }
            }
        });
        
        foundHipLinks.forEach(link => {
            if (!hipLinks.includes(link)) {
                hipLinks.push(link);
                const initialRotation = link.rotation.y;
                hipLinkInitialRotations.set(link, initialRotation);
            }
        });
        
        foundShoulderLinks.forEach(link => {
            if (!shoulderLinks.includes(link)) {
                shoulderLinks.push(link);
                const initialRotation = link.rotation.y;
                shoulderLinkInitialRotations.set(link, initialRotation);
            }
        });
        
        robot.updateMatrixWorld(true);
    }
}

const groundGeometry = new THREE.PlaneGeometry(50, 50);
const groundMaterial = new THREE.MeshStandardMaterial({ 
    color: 0x333333,
    roughness: 0.8,
    metalness: 0.2
});
const ground = new THREE.Mesh(groundGeometry, groundMaterial);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.5;
ground.receiveShadow = true;
scene.add(ground);

function updateCalfAnimation(delta) {
    if (calfLinks.length === 0) return;
    
    animationCycle += delta / ANIMATION_DURATION;
    if (animationCycle >= 2.0) animationCycle -= 2.0;
    
    const angle30Deg = Math.PI / 6;
    const angle45Deg = Math.PI / 4;
    const minAngle = angle30Deg * 2;
    const maxAngle = angle45Deg * 2;
    
    const phase1Progress = animationCycle < 1.0 ? animationCycle : 1.0;
    const phase2Progress = animationCycle >= 1.0 ? animationCycle - 1.0 : 0.0;
    
    const smoothProgress1 = Math.sin(phase1Progress * Math.PI);
    const smoothProgress2 = Math.sin(phase2Progress * Math.PI);
    
    const angle1 = minAngle + (maxAngle - minAngle) * smoothProgress1;
    const angle2 = minAngle + (maxAngle - minAngle) * smoothProgress2;
    
    calfLinks.forEach((link) => {
        if (!link) return;
        
        const linkName = (link.name || '').toLowerCase();
        const isFrontLeft = linkName.includes('fl_');
        const isBackRight = linkName.includes('rr_') || linkName.includes('hr_');
        const isFrontRight = linkName.includes('fr_');
        const isBackLeft = linkName.includes('rl_') || linkName.includes('hl_');
        
        let targetAngle = minAngle;
        
        if (isFrontLeft || isBackRight) {
            targetAngle = angle1;
        } else if (isFrontRight || isBackLeft) {
            targetAngle = angle2;
        }
        
        const initialRotation = calfLinkInitialRotations.get(link) || 0;
        link.rotation.y = initialRotation - targetAngle;
        link.updateMatrixWorld(true);
    });
    
    const hipSwingAmount = 0.6;
    const hipSwing1 = smoothProgress1 * hipSwingAmount;
    const hipSwing2 = smoothProgress2 * hipSwingAmount;
    
    hipLinks.forEach((link) => {
        if (!link) return;
        
        const linkName = (link.name || '').toLowerCase();
        const isFrontLeft = linkName.includes('fl_');
        const isBackRight = linkName.includes('rr_') || linkName.includes('hr_');
        const isFrontRight = linkName.includes('fr_');
        const isBackLeft = linkName.includes('rl_') || linkName.includes('hl_');
        
        let swingAngle = 0;
        
        if (isFrontLeft || isBackRight) {
            swingAngle = hipSwing1;
        } else if (isFrontRight || isBackLeft) {
            swingAngle = hipSwing2;
        }
        
        const initialRotation = hipLinkInitialRotations.get(link) || 0;
        link.rotation.y = initialRotation + swingAngle;
        link.updateMatrixWorld(true);
    });
    
    shoulderLinks.forEach((link) => {
        if (!link) return;
        
        const linkName = (link.name || '').toLowerCase();
        const isFrontLeft = linkName.includes('fl_');
        const isBackRight = linkName.includes('rr_') || linkName.includes('hr_');
        const isFrontRight = linkName.includes('fr_');
        const isBackLeft = linkName.includes('rl_') || linkName.includes('hl_');
        
        let swingAngle = 0;
        
        if (isFrontLeft || isBackRight) {
            swingAngle = -hipSwing1;
        } else if (isFrontRight || isBackLeft) {
            swingAngle = -hipSwing2;
        }
        
        const initialRotation = shoulderLinkInitialRotations.get(link) || 0;
        link.rotation.y = initialRotation + swingAngle;
        link.updateMatrixWorld(true);
    });
}

function animate() {
    requestAnimationFrame(animate);
    
    const delta = clock.getDelta();
    updateCalfAnimation(delta);
    
    if (robot) {
        robot.position.copy(robotInitialPosition);
        robot.updateMatrixWorld(true);
    }
    
    controls.update();
    renderer.render(scene, camera);
}

animate();


window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
