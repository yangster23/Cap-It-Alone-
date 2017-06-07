# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.shortcuts import get_object_or_404, render
from django.http import HttpResponseRedirect
from django.urls import reverse
from django.views import generic

from .models import User
from .forms import SignupForm


class SignupView(generic.FormView):
    template_name = 'leap/signup.html'
    form_class = SignupForm

    def get_context_data(self, **kwargs):
        context = super(SignupView, self).get_context_data(**kwargs)
        # context['user'] = User.objects.get(pk=int(self.kwargs['pk']))
        return context

    def form_valid(self, form):
        return super(SignupView, self).form_valid(form)

def signupform(request):
    if request.method == 'POST':
        form = SignupForm(request.POST)
        if form.is_valid():
            username_val = form.cleaned_data['username']
            user = User(username=username_val)
            user.save()
            return HttpResponseRedirect(reverse('leap:authentication', args=(user_id, 1)))
    else:
        form = SignupForm()
    return render(request, 'leap/signup.html', {'form':form});

class AuthenticationView(generic.DetailView):
    template_name = 'leap/authentication.html'
    model = User

    def get_context_data(self, **kwargs):
        context = super(AuthenticationView, self).get_context_data(**kwargs)
        return context
