<!DOCTYPE html>
{% raw %}
<html>
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Mobil Runtimemodel</title>
    <style>
      table, th, td {
        border:1px solid black;
      }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/jquery/3.5.1/jquery.min.js" integrity="sha512-bLT0Qm9VnAYZDflyKcBaQ2gg0hSYNQrJ8RilYldYQ1FxQYoCLtUjuuRuZo+fjqhx/qtq/1itJ0C2ejDxltZVFg=="crossorigin="anonymous"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/3.0.4/socket.io.js" integrity="sha512-aMGMvNYu8Ue4G+fHa359jcPb1u+ytAF+P2SCb+PxrjCdO3n3ZTxJ30zuH39rimUggmTwmh2u7wvQsDTHESnmfQ=="crossorigin="anonymous"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.7.1/chart.min.js"></script>
    <link
      href="//mincss.com/entireframework.min.css"
      rel="stylesheet"
      type="text/css"
    />
    <link href="{{url_for('static', filename = 'css/app.css')}}" rel="stylesheet">
  </head>
  <body>

    <script>
      //connect to the socket server.
      //   var socket = io.connect("http://" + document.domain + ":" + location.port);
      var socket = io.connect();

      // send goal to Client
      function goal() { 
        const selects = document.querySelectorAll('select');
            const formData = {}
            selects.forEach(input => {
              console.log(input.value);
              formData[(input.name)]= input.value;
            });
            socket.emit("formData", formData);
      } 
      // Code to be run only if the page DOM is ready for JS to execute
{% endraw %}
    {%- for c in classes -%}
        {% if c.name == "Robot" %}
        $(document).ready(function () {

            function addDataRobot(robot) {
                {% for feature in c.eStructuralFeatures -%}
                document.getElementById(robot.name+"{{feature.name}}").innerHTML = robot.{{feature.name}};
                {% endfor %}
            }
        
            //receive details from server
            socket.on("updateSensorData", function (msg) {
              data = JSON.parse(msg)
              for(let robot in data){
                addDataRobot(data[robot]);
              }
            });
          });
        {%-endif%}
    {%-endfor%}
    </script>

    {%- for c in classes -%}
        {% if c.name == "Robot" %}
                <table>
                    <tr>
                        {% for feature in c.eStructuralFeatures -%}
                            <th>{{feature.name}}</th>
                        {% endfor %}
                    </tr>
                    {{'{%for robot in context["robots"]%}'}}
                    <tr>
                        {% for feature in c.eStructuralFeatures -%}
                            <td id="{{'{{robot}}'}}{{feature.name}}">Hello World!</td>  
                        {% endfor %}
                    </tr>
                    {{'{%endfor%}'}}
                </table>

            {% raw %}
                <label for="robot">Choose a robot:</label>
                <select id="robot" name="robot" form="goal">
                  {%for robot in context["robots"]%}
                    <option value="{{robot}}">{{robot}}</option>
                  {%endfor%}
                </select>
            {% endraw %}



            {% for a in c.eReferences -%}
                <label for="{{a.name}}">Target{{a.name}}</label>
                <select id="{{a.name}}" name="{{a.name}}">
                {{'{% for'}} attribute in context["{{a.name}}s"] {{'%}'}}
                    {{'<option value="{{attribute}}">{{attribute}}</option>'}}
                {{'{% endfor %}'}}
                </select>
            {% endfor %}
                <button id="goalAdaption" onclick = "goal()"> Start Goal Adaption</button> 
        {%-endif%}
    {%-endfor%}
       
  </body>
</html>


