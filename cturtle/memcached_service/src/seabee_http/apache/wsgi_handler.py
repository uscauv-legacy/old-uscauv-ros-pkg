#!/usr/bin/env python
import os, sys

path = '/home/mmontalbo/workspace/seabee3-ros-pkg/memcached_service/src/seabee_http'
if path not in sys.path:
    sys.path.append(path)

path = '/home/mmontalbo/workspace/seabee3-ros-pkg/memcached_service/src'
if path not in sys.path:
    sys.path.append(path)

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/..')
os.environ['DJANGO_SETTINGS_MODULE'] = 'seabee_http.settings'

import django.core.handlers.wsgi

application = django.core.handlers.wsgi.WSGIHandler()
