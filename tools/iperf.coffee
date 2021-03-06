#!/usr/bin/env coffee

fs              = require 'fs'
{spawn, exec}   = require 'child_process'
path            = require 'path'

last = undefined

strim = (s) ->
  start = 0
  end = s.length
  for i in [0..s.length]
    if /\S/.test s.charAt(i) then start = i; break
  
  for i in [s.length..0]
    if /\S/.test s.charAt(i) then end = i; break

  if end < start then return '' else return s.substring(start, end + 1)


findIF = (devName) ->
  lines = fs.readFileSync('/proc/net/dev').toString().split('\n')
  return line for line in lines[2..] when line.search(devName) != -1


extractDevData = (data) ->
  data = data.split(':')[1]
  data = strim data
  data.split(/\s+/)


handleData = (arr) ->
  if last is undefined
    last = arr
  else
    mbps = arr[8] / 1024 / 1024
    rxNull = arr[14] - last[14]
    txNull = arr[23] - last[23]
    console.log "#{mbps},#{rxNull},#{txNull}"
    last = arr


main = ->
  if process.argv.length <= 2
    console.error "usage: ./#{path.basename process.argv[1]} [net device] [other iperf args...]"
    process.exit 1

  args = process.argv[3..]
  args.push '-yc'
  args.push '-i1'
  iperf = spawn 'iperf', args #['-u', '-c', '1.1.1.1', '-i1', '-t10', '-yc']

  devName = process.argv[2]

  raw = findIF(devName)
  if raw is undefined
    console.error "netdevice `#{devName}` not found"
    process.exit 1

  iperf.stdout.on 'data', (data) ->
    raw = findIF(devName)
    if raw is undefined
      console.error "netdevice `#{devName}` not found"

    devData = extractDevData(raw)
    data = strim data.toString()
    arrData = data.split(',')
    arrData = arrData.concat devData
    handleData arrData

  iperf.stderr.on 'data', (data) ->
    console.error data.toString()


if require.main is module then main()
