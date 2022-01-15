from flask import Flask, redirect, render_template, url_for
import yaml

app = Flask(__name__)

# looks for urls.yml
# load urls.yml into the dictionary

URL_FILE = 'urls.yml'
url_list = []

# url = yaml.load(URL_FILE)
# urls = [{"name":"",}]

def load_urls():
    """ loads the urls from the yaml file """
    with open(URL_FILE, "r") as stream:
        try:
            url_list = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    print(type(url_list))
    return url_list

def list_pretty_urls():
    """ prints the urls to the console """
    for item in url_list:
        print(item, url_list[item])
    

@app.route("/")
def index():
    return render_template("index.html")

@app.route('/<url>')
def pretty_route(url):
    """ Reroute URLS """

    # check if the url is in the list
    if url_list.get(url):
        print("Found", url)
        return redirect(url_list[url])
    else:
        # return "sorry no url of that name"
        print("not found", url)
        return render_template('404.html', url=url)

if "__name__" == "__main__":
    app.run(host="0.0.0.0", port="2222")

url_list = load_urls()
# print(url_list)

list_pretty_urls()
app.run(host="0.0.0.0", port="2222")