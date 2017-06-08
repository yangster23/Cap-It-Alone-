from django.conf.urls import url

from . import views

app_name = 'leap'
urlpatterns = [
    url(r'^signupform/$', views.signupform, name='signupform'),
    url(r'^$', views.SignupView.as_view(), name='signup'),
    url(r'^authentication/$', views.authentication, name='authentication'),
]
