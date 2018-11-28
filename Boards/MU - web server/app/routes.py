import os
from flask import render_template, flash, redirect, request
from app import app
from app.forms import LoginForm
from app.models import TractionStatus
from werkzeug import secure_filename

ALLOWED_EXTENSIONS = set(['rte'])
BASE_DIR = 'C:/Users/Yves1812/Documents/GitHub/Robot_tondeuse/Data'

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/')
@app.route('/index')
def index():
    user = {'username': 'Yves'}
    return render_template('index.html', title='Tondeuse autonome tout terrain', user=user)

@app.route('/login', methods=['GET', 'POST'])
def login():
    form = LoginForm()
    if form.validate_on_submit():
        flash('Login requested for user {}, remember_me={}'.format(
            form.username.data, form.remember_me.data))
        return redirect(url_for('index'))
    return render_template('login.html', title='Sign In', form=form)

@app.route('/traction_status', methods=['GET', 'POST'])
def traction_status():
    status = TractionStatus.query.order_by('timestamp desc').limit(1)
    if status is not None:
        parameters=dict((col, getattr(status[0],col)) for col in status[0].__table__.columns.keys())
    loaded_route="Aucune"
    if request.method == 'POST':
        try:
            loaded_route=request.form['route']
        except:
            pass
    return render_template('traction_status.html', title='Traction Status', parameters=parameters, loaded_route=loaded_route)
    
# Need to have this a page + then share the active file back with traction_status and let the backend know
@app.route('/select_route')
def select_route():
    abs_path = BASE_DIR
    # Return 404 if path doesn't exist
    if not os.path.exists(abs_path):
        return abort(404)

    # Check if path is a file and serve
    if os.path.isfile(abs_path):
        return send_file(abs_path)

    # Show directory contents
    files = os.listdir(abs_path)
    return render_template('select_route.html', files=files)

@app.route('/upload_route', methods = ['GET', 'POST'])
def upload_route():
    if request.method == 'POST':
        # check if the post request has the file part
        if 'file' not in request.files :
            flash('No file part')
            return redirect(request.url)
        file = request.files['file']
        # if user does not select file, browser also
        # submit an empty part without filename
        if file.filename == '':
            flash('No selected file')
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
        return redirect(url_for('traction_status'))
    return render_template('upload_route.html', title='Upload Route')
