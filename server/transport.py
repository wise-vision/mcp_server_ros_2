from mcp.server.fastmcp import FastMCP
from mcp.server.lowlevel.server import Server as BaseServer
from mcp.server.sse import SseServerTransport
from mcp.server.stdio import stdio_server
from starlette.applications import Starlette
from starlette.routing import Route, Mount
import uvicorn
import anyio


class TransportMixin:
    def __init__(
        self,
        server: BaseServer,
        *,
        host: str = "127.0.0.1",
        port: int = 8123,
        log_level: str = "info",
    ):
        self._mcp_server = server
        self._host = host
        self._port = port
        self._log_level = log_level

    async def run_stdio_async(self) -> None:
        async with stdio_server() as (read_stream, write_stream):
            await self._mcp_server.run(
                read_stream,
                write_stream,
                self._mcp_server.create_initialization_options(),
            )

    async def run_sse_async(
        self, sse_path: str = "/sse", message_path: str = "/messages"
    ) -> None:
        sse = SseServerTransport(message_path)

        async def handle_sse(scope, receive, send):
            async with sse.connect_sse(scope, receive, send) as streams:
                await self._mcp_server.run(
                    streams[0],
                    streams[1],
                    self._mcp_server.create_initialization_options(),
                )

        routes = [
            Route(sse_path, endpoint=handle_sse, methods=["GET"]),
            Mount(message_path, app=sse.handle_post_message),
        ]

        app = Starlette(debug=True, routes=routes)
        config = uvicorn.Config(
            app, host=self._host, port=self._port, log_level=self._log_level.lower()
        )
        server = uvicorn.Server(config)
        await server.serve()

    def run(self, transport: str = "stdio") -> None:
        if transport == "stdio":
            anyio.run(self.run_stdio_async)
        elif transport == "sse":
            anyio.run(lambda: self.run_sse_async())
        else:
            raise ValueError(f"Unsupported transport: {transport}")
