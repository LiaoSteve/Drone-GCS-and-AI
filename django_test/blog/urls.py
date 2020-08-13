from django.urls import path
from . import views

urlpatterns = [
    # Copter SSE
    path('', views.home, name='blog-home'),
    path('eventsource/', views.eventsource, name='event'), 
    # Socket webcam
    path('webcam/', views.web_cam, name='web_cam'),   
    # My introduction    
    path('about/', views.about, name='about'),
    # SSE data-stream
    path('chart/', views.chart, name='chart_data'),
    path('chart_eventsource/', views.chart_data_eventsource, name='chart_data'),
    # Test mapbox 
    path('mapbox/', views.mapbox, name='mapbox'),
    # Drone control system
    path('control/', views.drone_control, name='drone_control')
]
