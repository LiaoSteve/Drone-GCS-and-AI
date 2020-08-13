"""django_test URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/3.0/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.contrib import admin
from django.urls import path, include
from django.http import StreamingHttpResponse
from camera import VideoCamera, gen

urlpatterns = [    
    path('admin/', admin.site.urls),
    path('', include('blog.urls')),       
    #local cam
    path('yinyincam/', lambda r: StreamingHttpResponse(gen(VideoCamera(0)),content_type='multipart/x-mixed-replace; boundary=frame')),
    #path('cam/', lambda r: StreamingHttpResponse(gen(VideoCamera(0)),content_type='multipart/x-mixed-replace; boundary=frame')),
]
