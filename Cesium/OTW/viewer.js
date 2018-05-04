// Requires CesiumJS version 1.44
// Requires QtJSBSim code from 12 Apr 2018 or newer

// A key can be obtained from: https://www.bingmapsportal.com/
Cesium.BingMapsApi.defaultKey = 'your_key_here';

var viewer = new Cesium.Viewer('cesiumContainer', {
     terrainProvider : Cesium.createWorldTerrain(),
     baseLayerPicker : false,
     fullscreenButton : false,
     geocoder : false,
     homeButton : false,
     infoBox : false,
     sceneModePicker : false,
     selectionIndicator : false,
     timeline : false,
     navigationHelpButton : false,
     navigationInstructionsInitiallyVisible : false,
     animation: false
});

var FT2M = 0.3048;
var M2FT = 3.28083989501312;
var DEG2RAD = 0.017453293;

// pilot (camera)
var look_down_angle = 0.0;
var Hstep = 45.0;
var Vstep = 5.0;

// default aircraft orientation
var hdg_true_deg = 264.6;
var pitch_deg = 0.0;
var roll_deg = 0.0;

// default aircraft position
// LIMMA - final approach fix at LAX RWY 25L
var lon_deg = -118.273731;
var lat_deg = 33.9484444;
var alt_ft = 1900.0;


// set default camera view
viewer.camera.setView({
     destination : Cesium.Cartesian3.fromDegrees(lon_deg, lat_deg, (alt_ft * FT2M)),
     orientation : {
       heading : (hdg_true_deg * DEG2RAD),
       pitch: (pitch_deg * DEG2RAD),
       roll : (roll_deg * DEG2RAD)
     }
});


// set the camera using values from JSBSim via QtJSBSim
function updateCamera() {
     var cameraH = aircraft.hdg_true() + (aircraft.camBiasH() * Hstep);
     var cameraV = aircraft.pitch() + look_down_angle + (aircraft.camBiasV() * Vstep);
     viewer.camera.setView({
         destination : Cesium.Cartesian3.fromDegrees(aircraft.lon(), aircraft.lat(), (aircraft.alt() * FT2M)),
         orientation : {
           heading : (cameraH * DEG2RAD),
           pitch: (cameraV * DEG2RAD),
           roll : (aircraft.roll() * DEG2RAD)
         }
     });
}


// update camera 20 times per second
var timer = setInterval(updateCamera, 50);
