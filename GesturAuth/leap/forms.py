from django import forms
from .models import User

class SignupForm(forms.ModelForm):
    Username = forms.CharField(label='Username', max_length=100)
    class Meta:
        model = User
        fields = []
