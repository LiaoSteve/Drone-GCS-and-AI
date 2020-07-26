from django.urls import path
from . import views
print('2. open blog/url.py\n')
urlpatterns = [
    path('', views.home, name='blog-home'),
    path('eventsource/', views.eventsource, name='blog-event'), 

    path('webcam/', views.web_cam, name='blog-web_cam'),       
    path('about/', views.about, name='blog-about'),

    path('chart/', views.chart, name='blog-chart_data'),
    path('chart_eventsource/', views.chart_data_eventsource, name='blog-chart_data')
]
