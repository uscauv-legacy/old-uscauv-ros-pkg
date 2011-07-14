(function() {
  var ConfigModel, SeaBeeFeedController, SeaBeeSensorsView, init, verticallyPositionHack;
  var __hasProp = Object.prototype.hasOwnProperty, __extends = function(child, parent) {
    for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; }
    function ctor() { this.constructor = child; }
    ctor.prototype = parent.prototype;
    child.prototype = new ctor;
    child.__super__ = parent.prototype;
    return child;
  };
  ConfigModel = (function() {
    __extends(ConfigModel, Backbone.Model);
    function ConfigModel() {
      ConfigModel.__super__.constructor.apply(this, arguments);
    }
    ConfigModel.prototype.initialize = function() {
      return this.set({
        'intPressure': 'N/A',
        'extPressure': 'N/A',
        'heading': 'N/A'
      });
    };
    return ConfigModel;
  })();
  SeaBeeSensorsView = (function() {
    __extends(SeaBeeSensorsView, Backbone.View);
    function SeaBeeSensorsView() {
      SeaBeeSensorsView.__super__.constructor.apply(this, arguments);
    }
    SeaBeeSensorsView.prototype.initialize = function() {
      this.model.view = this;
      $("#heading-display").val(this.model.get('heading'));
      $("#intPressure-display").val(this.model.get('intPressure'));
      return $("#extPressure-display").val(this.model.get('extPressure'));
    };
    SeaBeeSensorsView.prototype.events = {
      "click #manual-update": "render"
    };
    SeaBeeSensorsView.prototype.render = function() {
      this.model.set({
        'intPressure': 'update',
        'extPressure': 'update',
        heading: 'update'
      });
      $("#heading-display").val(this.model.get('heading'));
      $("#intPressure-display").val(this.model.get('intPressure'));
      return $("#extPressure-display").val(this.model.get('extPressure'));
    };
    return SeaBeeSensorsView;
  })();
  SeaBeeFeedController = (function() {
    __extends(SeaBeeFeedController, Backbone.Controller);
    function SeaBeeFeedController() {
      SeaBeeFeedController.__super__.constructor.apply(this, arguments);
    }
    SeaBeeFeedController.prototype.initialize = function() {
      var model, sensor_view;
      model = new ConfigModel;
      return sensor_view = new SeaBeeSensorsView({
        'el': $('body'),
        'model': model
      });
    };
    return SeaBeeFeedController;
  })();
  init = function() {
    var sbf1;
    sbf1 = new SeaBeeFeedController;
    return verticallyPositionHack;
  };
  verticallyPositionHack = function() {
    var viewH, winH;
    winH = $("body").height() / 2;
    viewH = $("div#content").height() / 2;
    if (winH - viewH > 0) {
      return $("div#content").css("margin-top", winH - viewH);
    }
  };
  $(document).ready(init);
  $(window).resize(verticallyPositionHack);
}).call(this);
