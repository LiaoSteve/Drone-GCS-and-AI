from django.urls import path
from . import views
urlpatterns = [
    path('', views.home, name='blog-home'),
    path('eventsource/', views.eventsource, name='blog-event'),
    path('about/', views.about, name='blog-about')
]
