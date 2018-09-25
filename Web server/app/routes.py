from flask import render_template, flash, redirect, request
from app import app
from app.forms import LoginForm
from app.models import TractionStatus
from werkzeug import secure_filename


@app.route('/')
@app.route('/index')
def index():
    user = {'username': 'Miguel'}
    posts = [
        {
            'author': {'username': 'John'},
            'body': 'Beautiful day in Portland!'
        },
        {
            'author': {'username': 'Susan'},
            'body': 'The Avengers movie was so cool!'
        }
    ]
    return render_template('index.html', title='Home', user=user, posts=posts)


@app.route('/login', methods=['GET', 'POST'])
def login():
    form = LoginForm()
    if form.validate_on_submit():
        flash('Login requested for user {}, remember_me={}'.format(
            form.username.data, form.remember_me.data))
        return redirect(url_for('index'))
    return render_template('login.html', title='Sign In', form=form)

@app.route('/traction_status')
def traction_status():
    status = TractionStatus.query.order_by('timestamp desc').limit(1)
    if status is not None:
        parameters=dict((col, getattr(status[0],col)) for col in status[0].__table__.columns.keys())
    return render_template('traction_status.html', title='Traction Status', parameters=parameters)

@app.route('/load_route')
def load_route():
    return render_template('load_route.html', title='Load Route')

@app.route('/uploader', methods = ['GET', 'POST'])
def upload_file():
   if request.method == 'POST':
      f = request.files['file']
      f.save(secure_filename(f.filename))
      return 'file uploaded successfully'
