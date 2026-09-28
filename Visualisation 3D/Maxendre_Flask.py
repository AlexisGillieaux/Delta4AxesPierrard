from flask import Flask

app = Flask(__name__)

@app.route("/")
def hello():
    # On utilise du HTML/CSS directement pour centrer et grossir le texte
    return """
    <html>
        <head>
            <title>Hello World</title>
            <style>
                body {
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                    font-family: Arial, sans-serif;
                }
                h1 {
                    font-size: 80px;
                }
            </style>
        </head>
        <body>
            <h1>Hello World</h1>
        </body>
    </html>
    """

if __name__ == "__main__":
    app.run(debug=True)