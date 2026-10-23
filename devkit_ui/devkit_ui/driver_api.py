"""Forward of the driver's HTTP API, so the robot answers ``/api/...`` on the UI port."""

import httpx
from fastapi import Response
from fastapi.responses import JSONResponse

DRIVER_API_URL = 'http://127.0.0.1:8080/api'
DRIVER_TIMEOUT = 5.0


async def forward_to_driver(path: str) -> Response:
    """Forward a GET request to the driver's API and return its answer unchanged.

    :param path: the path below ``/api/``, e.g. ``status``
    :return: the driver's response, or HTTP 503 if the driver cannot be reached
    """
    try:
        async with httpx.AsyncClient(timeout=DRIVER_TIMEOUT) as client:
            response = await client.get(f'{DRIVER_API_URL}/{path}')
    except httpx.HTTPError as error:
        return JSONResponse({'detail': f'driver unreachable: {error!r}'}, status_code=503)
    return Response(response.content, status_code=response.status_code, media_type=response.headers.get('content-type'))
