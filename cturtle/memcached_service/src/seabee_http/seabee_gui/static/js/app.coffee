class ConfigModel extends Backbone.Model
    initialize: ->
        @set 'intPressure': 'N/A', 'extPressure': 'N/A', 'heading': 'N/A'
        
class SeaBeeSensorsView extends Backbone.View
    initialize: ->
       @model.view = @
       $("#heading-display").val @model.get('heading')
       $("#intPressure-display").val @model.get('intPressure')
       $("#extPressure-display").val @model.get('extPressure')
    
    events: {
        "click #manual-update": "render"
    }

    render: ->
       @model.set 'intPressure':'update','extPressure':'update',heading:'update'
       $("#heading-display").val @model.get('heading')
       $("#intPressure-display").val @model.get('intPressure')
       $("#extPressure-display").val @model.get('extPressure')
       
class SeaBeeFeedController extends Backbone.Controller
    initialize: ->
       model = new ConfigModel
       sensor_view = new SeaBeeSensorsView 'el': $('body'), 'model': model
       
init = ->
    sbf1 = new SeaBeeFeedController
    verticallyPositionHack
    
verticallyPositionHack = ->
	winH = $("body").height() / 2
	viewH = $("div#content").height() / 2
	$("div#content").css("margin-top",winH - viewH) if winH - viewH > 0
    

$(document).ready init
$(window).resize verticallyPositionHack
