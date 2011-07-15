
EMPTY_VALUE = 'n/a'

class SeaBeeDataModel extends Backbone.Model
    defaults:
      'ext_pressure': EMPTY_VALUE,
      'int_pressure': EMPTY_VALUE,
      'heading': EMPTY_VALUE
    url: ->
        '/data/'

    start_updating: =>
      setInterval =>
       @fetch()
      ,300

class SeaBeeSensorsView extends Backbone.View
    initialize: ->
       @model.view = @
       $("#heading-display").val @model.get('heading')
       $("#intPressure-display").val @model.get('int_pressure')
       $("#extPressure-display").val @model.get('ext_pressure')

    events: {
        "click #manual-update": "render"
    }

    render: =>
       $("#heading-display").val @model.get('heading')
       $("#intPressure-display").val @model.get('int_pressure')
       $("#extPressure-display").val @model.get('ext_pressure')

class SeaBeeFeedController extends Backbone.Controller
    initialize: ->
       model = new SeaBeeDataModel
       model.start_updating()
       sensor_view = new SeaBeeSensorsView 'el': $('body'), 'model': model
       model.bind("change",sensor_view.render)


init = ->
    sbf1 = new SeaBeeFeedController
    verticallyPositionHack

verticallyPositionHack = ->
	winH = $("body").height() / 2
	viewH = $("div#content").height() / 2
	$("div#content").css("margin-top",winH - viewH) if winH - viewH > 0


$(document).ready init
$(window).resize verticallyPositionHack
