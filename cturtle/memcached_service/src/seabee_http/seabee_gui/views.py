from django.http import HttpResponse
from django.utils import simplejson as json
from django.shortcuts import render_to_response

import memcache

SERVICE_KEY = 'seabee3'

def main(request):
    return render_to_response('index.html',
                              None)
    
def data(request):
    try:
        mc = memcache.Client(['127.0.0.1:11211'], debug=0)
        if mc is not None:
            val = mc.get(SERVICE_KEY)
            return HttpResponse(val)
        else:
            return HttpResponseServerError("Error: Could not connect to memcache.")
    except Exception, e:
        return HttpResponseServerError('Error: '+str(e))
