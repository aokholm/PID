// requirements

var simulation;

function setup() {
  simulation = new Simulation()
  simulation.setup()
}

function Simulation() {
  this.updateInterval = 10 //ms
  this.canvas = document.createElement("canvas")
  this.canvasScale = 100 // 100 pixel per meter
  this.canvas.widthMeter = 4
  this.canvas.heightMeter = 4
  this.canvas.width = this.canvas.widthMeter * this.canvasScale
  this.canvas.height = this.canvas.heightMeter * this.canvasScale

  this.context = this.canvas.getContext("2d")

  this.pidUI = new createPIDUI()

  this.container = document.getElementById('simulation')

  this.container.appendChild(this.canvas)
  this.container.appendChild(document.createElement("br"))
  this.container.appendChild(createParagraph("control:"))
  this.container.appendChild(this.pidUI.container)
  this.container.appendChild(document.createElement("br"))

  this.container.appendChild(createButton("Start \"game\"", "simulation.start()"))
  this.container.appendChild(createButton("Stop  \"game\"", "simulation.stop()"))


  this.timerText = new TextComponent(this.canvas.width-200, 40, {"fontSize": "20px"})
  this.statusText = new TextComponent(20, 40, {"fontSize": "20px"})
  this.gameOverText = new TextComponent(this.canvas.width/2-100, this.canvas.height/2-30, {"text": "Game Over", "active": false})

  this.drawables = [this.timerText, this.gameOverText, this.statusText]

  this.datasets = []
  this.activeSet = []
}

Simulation.prototype = {

  setup : function() {
    this.frameNo = 0;
    this.activeSet = []
    var x = 2
    var y = 1 // initial position
    this.drone = new DroneComponent(x,y, this.canvasScale, this.pidUI)
    this.gameOverText.options.active = false
    this.draw()
  },

  start : function() {
    this.interval = setInterval(this.loop.bind(this), this.updateInterval)
  },

  stop : function() {
    clearInterval(this.interval)
  },

  pause : function(time) {
    return new Promise( function(resolve, reject) {
      setTimeout( function() {
        resolve()
      }, time)
    })
  },


  clear : function() {
    this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
  },

  loop : function() {

    this.updateLogic()
    this.draw()
  },

  updateLogic : function() {
    this.frameNo += 1

    this.drone.newPos()

    if (this.drone.outOfBounds(this.canvas)) {
      this.gameOver()
    }
    if (this.frameNo == 10000) {
      this.gameOver()
    }

    this.timerText.options.text = "SCORE: " + this.frameNo;
  },

  draw : function() {
    this.clear()
    this.drawables.map (function(drawable) {
      drawable.draw(this.context)
    }, this)

    this.drone.draw(this.context, this.canvas)

  },

  gameOver : function() {
    var self = this
    this.gameOverText.options.active = true
    clearInterval(this.interval)
    this.pause(2000)
    .then( function() {
      self.setup()
    })
  },
}


function DroneComponent(x, y, scale, pidUI) {

  this.yTarget = 2 // m
  this.scale = scale
  this.pidUI = pidUI

  this.yPID = new PID()

  this.x = x
  this.y = y
  this.dy = 0
  this.dx = 0

  this.m = 1 // kg

  this.gravity = -9.82 // m/s^2


  this.dt = 0.01 // time step

  // plotting
  this.width = 20
  this.height = 20
  this.color = "red"


  this.newPos = function() {

    // PID controller

    var error = this.yTarget - this.y;
    var yPIDresult = this.yPID.update(error)


    var thrust = this.pidUI.Kp() * yPIDresult.p + this.pidUI.Ki() * yPIDresult.i + this.pidUI.Kd() * yPIDresult.d

    this.thrustAcc = Math.min( Math.max( thrust * 20 / this.m, 0), 20)
    this.ddy = this.gravity + this.thrustAcc

    this.dy += this.ddy * this.dt
    this.y += this.dy * this.dt
  }


  this.draw = function(ctx, canvas) {
    ctx.fillStyle = this.color;
    ctx.save(); // save the unrotated context of the canvas so we can restore it later
    ctx.translate(this.x * this.scale, canvas.height - this.y * this.scale); // move to the point of the drone
    // ctx.rotate(this.directionUpsideDown); // rotate the canvas to the specified degrees

    // draw the drone
    ctx.beginPath();
    ctx.moveTo(-10, 10);
    ctx.lineTo(0,-10);
    ctx.lineTo(10, 10);
    ctx.closePath();
    ctx.fill();

    ctx.restore(); // we’re done with the rotating so restore the unrotated ctx
  }

  this.outOfBounds = function(canvas) {
    var mleft = this.x*this.scale - this.width/2
    var mright = this.x*this.scale + this.width/2
    var mtop = this.y*this.scale - this.height/2
    var mbottom = this.y*this.scale + this.height/2

    return (mleft < 0) || (mright > canvas.width) || (mtop < 0) || (mbottom > canvas.height)
  }
}


function PID() {
  this.listSize = 200
  this.buffer = []

  this.update = function(error) {
    // proportional
    var p = error
    this.buffer.push(error)

    // integral
    if (this.buffer.length > this.bufferSize) {
      this.buffer.pop()
    }

    var sum = 0;
    for(var i = 0; i < this.buffer.length; i++) {
        sum += this.buffer[i]
    }
    var i = sum / this.buffer.length


    // derivative
    var d = 0
    if (typeof this.lastError != 'undefined') {
      d = error - this.lastError
    }

    this.lastError = error

    return {
      p: p,
      i: i,
      d: d,
    }

  }
}

function TextComponent(x, y, options) {
  this.x = x || 0
  this.y = y || 0

  var defaults = {
      text: "empty",
      fontSize: "30px",
      fontStyle: "Consolas",
      color: "black",
      active: true
  }
  this.options = merge(defaults, options || {})

  this.draw = function(ctx) {
    if (this.options.active) {
      ctx.font = this.options.fontSize + " " + this.options.fontStyle;
      ctx.fillStyle = this.options.color;
      ctx.fillText(this.options.text, this.x, this.y);
    }
  }
}

function plotLine(ctx, line, color) {
    // draw the drone
    ctx.strokeStyle = "#000000";
    if (color) {
      ctx.strokeStyle = "#" + color;
    }

    ctx.lineWidth=1;
    ctx.beginPath();
    ctx.moveTo(line[0][0], line[0][1]);

    for (var i = 1; i < line.length; i++) {
      ctx.lineTo(line[i][0], line[i][1]);
    }
    ctx.stroke();
}

function merge() {
    var obj, name, copy,
        target = arguments[0] || {},
        i = 1,
        length = arguments.length;

    for (; i < length; i++) {
        if ((obj = arguments[i]) != null) {
            for (name in obj) {
                copy = obj[name];

                if (target === copy) {
                    continue;
                }
                else if (copy !== undefined) {
                    target[name] = copy;
                }
            }
        }
    }

    return target;
}

function createPIDUI() {
  this.container = document.createElement("div")
  this.pSlider = createSlider()
  this.iSlider = createSlider()
  this.dSlider = createSlider()
  this.container.appendChild(this.pSlider)
  this.container.appendChild(document.createElement("br"))
  this.container.appendChild(this.iSlider)
  this.container.appendChild(document.createElement("br"))
  this.container.appendChild(this.dSlider)

  this.Kp = function() {
    return this.pSlider.value / 200
  }

  this.Ki = function() {
    return this.iSlider.value / 200
  }

  this.Kd = function() {
    return this.dSlider.value / 200
  }
}


function createSlider(options) {
  var slider = document.createElement("input")
  var defaults = {
    "type": "range",
    "min": 0,
    "max": 1000,
    "step": 1,
    "style": "width:400px"
  }
  options = merge(defaults, options || {})

  Object.keys(options).forEach( function(key) {
    slider.setAttribute(key, options[key])
  })
  return slider
}

function createParagraph(text) {
  var p = document.createElement("p")
  p.innerHTML = text
  return p
}

function createButton(text, action) {
  var button = document.createElement("button")
  button.setAttribute("onclick", action)
  button.innerHTML = text
  return button
}
