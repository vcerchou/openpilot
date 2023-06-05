#!/usr/bin/env python3
<<<<<<< HEAD
from flask import Flask, render_template, Response, request
from system.dashcamviewer.helpers import *
=======
import os
import system.dashcamviewer.helpers as dashcam
from flask import Flask, render_template, Response, request, send_from_directory
from system.loggerd.config import ROOT as REALDATA
>>>>>>> 113316d4e8502f2c3065c7594a74c8741e640c29

app = Flask(__name__)


@app.route("/")
def index_page():
  return render_template("index.html")


@app.route("/footage/full/<cameratype>/<route>")
def full(cameratype, route):
  chunk_size = 1024 * 512  # 5KiB
  file_name = cameratype + (".ts" if cameratype == "qcamera" else ".hevc")
<<<<<<< HEAD
  vidlist = "|".join(ROOT + "/" + segment + "/" + file_name for segment in segments_in_route(route))

  def generate_buffered_stream():
    with ffmpeg_mp4_concat_wrap_process_builder(vidlist, cameratype, chunk_size) as process:
=======
  vidlist = "|".join(REALDATA + "/" + segment + "/" + file_name for segment in dashcam.segments_in_route(route))

  def generate_buffered_stream():
    with dashcam.ffmpeg_mp4_concat_wrap_process_builder(vidlist, cameratype, chunk_size) as process:
>>>>>>> 113316d4e8502f2c3065c7594a74c8741e640c29
      for chunk in iter(lambda: process.stdout.read(chunk_size), b""):
        yield bytes(chunk)
  return Response(generate_buffered_stream(), status=200, mimetype='video/mp4')


@app.route("/footage/<cameratype>/<segment>")
def fcamera(cameratype, segment):
<<<<<<< HEAD
  if not is_valid_segment(segment):
    return "invalid segment"
  file_name = ROOT + "/" + segment + "/" + cameratype + (".ts" if cameratype == "qcamera" else ".hevc")

  return Response(ffmpeg_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')
=======
  if not dashcam.is_valid_segment(segment):
    return render_template("error.html", error="invalid segment")
  file_name = REALDATA + "/" + segment + "/" + cameratype + (".ts" if cameratype == "qcamera" else ".hevc")
  return Response(dashcam.ffmpeg_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')
>>>>>>> 113316d4e8502f2c3065c7594a74c8741e640c29


@app.route("/footage/<route>")
def route(route):
  if len(route) != 20:
<<<<<<< HEAD
    return "route not found"
=======
    return render_template("error.html", error="route not found")
>>>>>>> 113316d4e8502f2c3065c7594a74c8741e640c29

  if str(request.query_string) == "b''":
    query_segment = str("0")
    query_type = "qcamera"
  else:
    query_segment = (str(request.query_string).split(","))[0][2:]
    query_type = (str(request.query_string).split(","))[1][:-1]

  links = ""
  segments = ""
<<<<<<< HEAD
  for segment in segments_in_route(route):
    links += "<a href='"+route+"?"+segment.split("--")[2]+","+query_type+"'>"+segment+"</a><br>"
    segments += "'"+segment+"',"
  return """<html>
  <head>
    <meta name="viewport" content="initial-scale=1, width=device-width"/>
    <link href="/static/favicon.ico" rel="icon">
    <title>Dashcam Footage</title>
  </head>
  <body>
  <center>
    <video id="video" width="320" height="240" controls autoplay="autoplay" style="background:black">
    </video>
    <br><br>
    current segment: <span id="currentsegment"></span>
    <br>
    current view: <span id="currentview"></span>
    <br>
    <a download=\""""+route+"-"+ query_type + ".mp4" + """\" href=\"/footage/full/"""+query_type+"""/"""+route+"""\">download full route """ + query_type + """</a>
    <br><br>
    <a href="/footage">back to routes</a>
    <br><br>
    <a href=\""""+route+"""?0,qcamera\">qcamera</a> -
    <a href=\""""+route+"""?0,fcamera\">fcamera</a> -
    <a href=\""""+route+"""?0,dcamera\">dcamera</a> -
    <a href=\""""+route+"""?0,ecamera\">ecamera</a>
    <br><br>
    """+links+"""
  </center>
  </body>
    <script>
    var video = document.getElementById('video');
    var tracks = {
      list: ["""+segments+"""],
      index: """+query_segment+""",
      next: function() {
        if (this.index == this.list.length - 1) this.index = 0;
        else {
            this.index += 1;
        }
      },
      play: function() {
        return ( \""""+query_type+"""/" + this.list[this.index] );
      }
    }
    video.addEventListener('ended', function(e) {
      tracks.next();
      video.src = tracks.play();
      document.getElementById("currentsegment").textContent=video.src.split("/")[5];
      document.getElementById("currentview").textContent=video.src.split("/")[4];
      video.load();
      video.play();
    });
    video.src = tracks.play();
    document.getElementById("currentsegment").textContent=video.src.split("/")[5];
    document.getElementById("currentview").textContent=video.src.split("/")[4];
    </script>
</html>
"""


@app.route("/footage")
def index():
  result = """
  <html>
    <head>
      <meta name="viewport" content="initial-scale=1, width=device-width"/>
      <link href="/static/favicon.ico" rel="icon">
      <title>Dashcam Footage</title>
    </head>
    <body><center><br><a href='\\'>Back to landing page</a><br>"""
  for route in all_routes():
    result += "<a href='footage/"+route+"'>"+route+"</a><br>"
  result += """</center></body></html>"""
  return result


def main():
  app.run(host="0.0.0.0")
=======
  for segment in dashcam.segments_in_route(route):
    links += "<a href='"+route+"?"+segment.split("--")[2]+","+query_type+"'>"+segment+"</a><br>"
    segments += "'"+segment+"',"

  return render_template("route.html", route=route, query_type=query_type, links=links, segments=segments, query_segment=query_segment)

@app.route("/footage")
def footage():
  return render_template("footage.html", rows=dashcam.all_routes())


@app.route("/screenrecords")
def screenrecords():
  rows = dashcam.all_screenrecords()
  if not rows:
    return render_template("error.html", error="no screenrecords found at:<br><br>" + dashcam.SCREENRECORD_PATH)
  return render_template("screenrecords.html", rows=rows, clip=rows[0])


@app.route("/screenrecords/<clip>")
def screenrecord(clip):
  return render_template("screenrecords.html", rows=dashcam.all_screenrecords(), clip=clip)


@app.route("/screenrecords/play/pipe/<file>")
def videoscreenrecord(file):
  file_name = dashcam.SCREENRECORD_PATH + file
  return Response(dashcam.ffplay_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')


@app.route("/screenrecords/download/<clip>")
def download_file(clip):
  return send_from_directory(dashcam.SCREENRECORD_PATH, clip, as_attachment=True)


@app.route("/about")
def about():
  return render_template("about.html")


def main():
  app.run(host="0.0.0.0", port=5050)
>>>>>>> 113316d4e8502f2c3065c7594a74c8741e640c29


if __name__ == '__main__':
  main()