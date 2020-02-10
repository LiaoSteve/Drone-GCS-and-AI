from django.urls import path
from . import views
print('2. open blog/url.py\n')
urlpatterns = [
    path('', views.home, name='blog-home'),
    path('webcam/', views.web_cam, name='blog-web_cam'),
    path('eventsource/', views.eventsource, name='blog-event'),    
    path('about/', views.about, name='blog-about')
]
