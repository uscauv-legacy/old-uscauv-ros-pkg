from django.conf.urls.defaults import patterns, include, url

urlpatterns = patterns('',
    url(r'^$', 'seabee_gui.views.main'),
    url(r'^data/$', 'seabee_gui.views.data'),
)

urlpatterns += patterns('',
        (r'^static/(?P<path>.*)$', 'django.views.static.serve', {'document_root': '/home/mmontalbo/workspace/seabee3-ros-pkg/memcached_service/src/seabee_http/seabee_gui/static'}),
    )
