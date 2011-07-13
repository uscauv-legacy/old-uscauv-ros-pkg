#!/usr/bin/env python
import os, sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/..')
os.environ['DJANGO_SETTINGS_MODULE'] = 'example_com.settings'

import django.core.handlers.wsgi

application = django.core.handlers.wsgi.WSGIHandler()
