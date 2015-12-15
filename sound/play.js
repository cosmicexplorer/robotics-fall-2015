var flock = require('flocking');
var enviro = flock.init({
  chans: 8
});

var midC = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 261.625565
  },
  addToEnvironment: false
});


var D = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 293.665
  },
  addToEnvironment: false
});


var E = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 329.628
  },
  addToEnvironment: false
});


var F = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 348.228
  },
  addToEnvironment: false
});

var G = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 391.995
  },
  addToEnvironment: false
});

var A = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 440
  },
  addToEnvironment: false
});


var B = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 493.883
  },
  addToEnvironment: false
});


var upC = flock.synth({
  synthDef: {
    ugen: "flock.ugen.sinOsc",
    freq: 523.251
  },
  addToEnvironment: false
});

enviro.start();

function setTime(time, f) {
  setTimeout(f, time);
}

setTime(1000, function(){
  enviro.head(midC);
  setTime(1000, function(){
    enviro.head(E);
    setTime(1000, function(){
      enviro.remove(midC);
      setTime(1000, function(){
        enviro.stop();
      });
    });
  });
});
